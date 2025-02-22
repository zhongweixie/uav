#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <control_demo/common.h>
#include <map>
#include <algorithm>
using namespace nvinfer1;
using namespace std;
static Logger gLogger;
const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;


bool parse_args(string &yolov5_wtstype, float &gd, float &gw)
{

  if (yolov5_wtstype == "n")
  {
    gd = 0.33;
    gw = 0.25;
  }
  else if (yolov5_wtstype == "s")
  {
    gd = 0.33;
    gw = 0.50;
  }
  else if (yolov5_wtstype == "m")
  {
    gd = 0.67;
    gw = 0.75;
  }
  else if (yolov5_wtstype == "l")
  {
    gd = 1.0;
    gw = 1.0;
  }
  else if (yolov5_wtstype == "x")
  {
    gd = 1.33;
    gw = 1.25;
  }
  else
  {
    return false;
  }
  return true;
}

void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer, float **gpu_output_buffer, float **cpu_output_buffer)
{
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc((void **)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));

  *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void infer(IExecutionContext &context, cudaStream_t &stream, void **gpu_buffers, float *output, int batchsize)
{
  context.enqueue(batchsize, gpu_buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}

void serialize_engine(unsigned int max_batchsize, bool &is_p6, float &gd, float &gw, std::string &wts_name, std::string &engine_name)
{
  // Create builder
  IBuilder *builder = createInferBuilder(gLogger);
  IBuilderConfig *config = builder->createBuilderConfig();

  // Create model to populate the network, then set the outputs and create an engine
  ICudaEngine *engine = nullptr;
  if (is_p6)
  {
    engine = build_det_p6_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
  }
  else
  {
    engine = build_det_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
  }
  assert(engine != nullptr);

  // Serialize the engine
  IHostMemory *serialized_engine = engine->serialize();
  assert(serialized_engine != nullptr);

  // Save engine to file
  std::ofstream p(engine_name, std::ios::binary);
  if (!p)
  {
    std::cerr << "Could not open plan output file" << std::endl;
    assert(false);
  }
  p.write(reinterpret_cast<const char *>(serialized_engine->data()), serialized_engine->size());

  // Close everything down
  engine->destroy();
  builder->destroy();
  config->destroy();
  serialized_engine->destroy();
}

void deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine, IExecutionContext **context)
{
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good())
  {
    std::cerr << "read " << engine_name << " error!" << std::endl;
    assert(false);
  }
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  char *serialized_engine = new char[size];
  assert(serialized_engine);
  file.read(serialized_engine, size);
  file.close();

  *runtime = createInferRuntime(gLogger);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] serialized_engine;
}

class YOLOV5_DETECT
{
public:
  YOLOV5_DETECT(string wts_path, std::string engine_path, std::string yolov5_wtstype, bool make_engine = false)
  {
    cudaSetDevice(kGpuId);
    if (!parse_args(yolov5_wtstype, gd, gw))
    {
      std::cerr << "arguments yolov5_wtstype not right! [n/s/m/l/x]" << std::endl;
    }
    if (make_engine)
    {
      serialize_engine(kBatchSize, is_p6, gd, gw, wts_path, engine_path);
    }
    // Deserialize the engine from file
    deserialize_engine(engine_path, &runtime, &engine, &context);
    CUDA_CHECK(cudaStreamCreate(&stream));
    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);
    // Prepare cpu and gpu buffers
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);
  }
  ~YOLOV5_DETECT()
  {
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));
    delete[] cpu_output_buffer;
    cuda_preprocess_destroy();
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
  }
  std::vector<DETECT_RESULT> detect(cv::Mat &img, cv::Mat &imgout)
  {
    for (size_t i = 0; i < file_names.size(); i += kBatchSize)
    {
      // Get a batch of images
      std::vector<cv::Mat> img_batch;
      std::vector<std::string> img_name_batch;
      for (size_t j = i; j < i + kBatchSize && j < file_names.size(); j++)
      {
        img_batch.push_back(img);
        img_name_batch.push_back(file_names[j]);
      }

      // Preprocess
      cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

      // Run inference
      auto start = std::chrono::system_clock::now();
      infer(*context, stream, (void **)gpu_buffers, cpu_output_buffer, kBatchSize);
      auto end = std::chrono::system_clock::now();
      std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

      // NMS
      std::vector<std::vector<Detection>> res_batch;
      batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

      // Draw bounding boxes
      draw_bbox(img_batch, res_batch);

      // return result
      std::vector<DETECT_RESULT> result_vec;
      for (size_t j = 0; j < res_batch[0].size(); j++)
      {
        DETECT_RESULT result;
         result.Rect=get_rect(img_batch[0],res_batch[0][j].bbox);
        result.Id = (int)res_batch[0][j].class_id;
        if (result.Id == 39)
        {
          result.name = "obj_target1";
        }
        result.Det_type = "yolov5";
         result_vec.push_back(result);
      }
        imgout = img_batch[0];
        return result_vec;
      }
    }
    float gd = 0.0f, gw = 0.0f;
    bool is_p6 = false;
    IRuntime *runtime = nullptr;
    ICudaEngine *engine = nullptr;
    IExecutionContext *context = nullptr;
    cudaStream_t stream;
    float *gpu_buffers[2];
    float *cpu_output_buffer = nullptr;
    std::vector<std::string> file_names = {"image1"};
  };
