#include <ros/ros.h>
#include <control_demo/image_srv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yolov5_det.cpp>
#include <color_det.cpp>
#include <circle_det.cpp>
#include <map>
using namespace std;

struct SCALE
{
    std::string Name;
    float Wh[2];
};
int solve_pnp(cv::Rect rect, float map_wh[2])
{
    float width = map_wh[0];
    float height = map_wh[1];

    // 定义3D物体的坐标
    vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(-width / 2.0, -height / 2.0, 0));
    objectPoints.push_back(cv::Point3f(width / 2.0, -height / 2.0, 0));
    objectPoints.push_back(cv::Point3f(width / 2.0, height / 2.0, 0));
    objectPoints.push_back(cv::Point3f(-width / 2.0, height / 2.0, 0));

    // 定义2D图像中的坐标
    vector<cv::Point2f> imagePoints;
    // 将cv::Rect类型的四个顶点转换为vector<Point2f>
    imagePoints.push_back(cv::Point2f(rect.x, rect.y));
    imagePoints.push_back(cv::Point2f(rect.x + rect.width, rect.y));
    imagePoints.push_back(cv::Point2f(rect.x + rect.width, rect.y + rect.height));
    imagePoints.push_back(cv::Point2f(rect.x, rect.y + rect.height));

    // 定义相机内参矩阵
    double cameramatrix[3][3] = {{961.673333, 0.000000, 632.877909},
                                 {0.000000, 962.120607, 380.182187},
                                 {0.000000, 0.000000, 1.000000}};
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = cameramatrix[0][0];
    cameraMatrix.at<double>(1, 1) = cameramatrix[1][1];
    cameraMatrix.at<double>(0, 2) = cameramatrix[0][2];
    cameraMatrix.at<double>(1, 2) = cameramatrix[1][2];

    // 定义畸变系数
    double distcoeffs[1][5] = {0.020247, -0.092110, 0.001461, 0.000050, 0.000000};
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = distcoeffs[0][0];
    distCoeffs.at<double>(1, 0) = distcoeffs[0][1];
    distCoeffs.at<double>(2, 0) = distcoeffs[0][2];
    distCoeffs.at<double>(3, 0) = distcoeffs[0][3];
    distCoeffs.at<double>(4, 0) = distcoeffs[0][4];
    // 定义旋转向量和平移向量
    cv::Mat rvec, tvec;

    // 使用solvePnP函数进行PnP解算
    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    // 输出旋转向量和平移向量
    // cout << "rvec: " << rvec << endl;
    // cout << "tvec: " << tvec << endl;

    return 0;
}

class IMAGE_PROSESS
{
public:
    IMAGE_PROSESS(ros::NodeHandle &nh, cv::Mat &frame) : Frame(frame), Yolov5_detect("/home/zs/turtle_ws/src/control_demo/weights/yolov5s.wts", "/home/zs/turtle_ws/src/control_demo/weights/yolov5s.engine", "s", false)
    {
        ros::ServiceServer service = nh.advertiseService("image_server", &IMAGE_PROSESS::image_server_callback, this);
        SCALE scale = {"obj_target1", {0.1, 0.3}};
        Scale_vec.push_back(scale);
        SCALE scale1 = {"obj_target2", {0.2, 0.6}};
        Scale_vec.push_back(scale1);
        SCALE scale2 = {"red", {0.2, 0.6}};
        Scale_vec.push_back(scale2);
        SCALE scale3 = {"green", {0.2, 0.6}};
        Scale_vec.push_back(scale3);
        SCALE scale4 = {"circle1", {0.1, 0.3}};
        Scale_vec.push_back(scale4);
        SCALE scale5 = {"circle2", {0.2, 0.6}};
        Scale_vec.push_back(scale5);
    }
    static void on_MouseHandle(int event, int x, int y, int flags, void *param)
    {
        cv::Mat srcImage = *(cv::Mat *)param;
        // cv::imshow("test", srcImage);
        switch (event)
        {
        case CV_EVENT_LBUTTONDOWN:
        {
            cout << "B: " << (int)srcImage.at<cv::Vec3b>(y, x)[0] << '\t';
            cout << "G: " << (int)srcImage.at<cv::Vec3b>(y, x)[1] << '\t';
            cout << "R: " << (int)srcImage.at<cv::Vec3b>(y, x)[2] << endl;

            cv::Mat dstImage;
            cv::cvtColor(srcImage, dstImage, cv::COLOR_BGR2HSV);
            cout << "H: " << (int)dstImage.at<cv::Vec3b>(y, x)[0] << '\t';
            cout << "S: " << (int)dstImage.at<cv::Vec3b>(y, x)[1] << '\t';
            cout << "V: " << (int)dstImage.at<cv::Vec3b>(y, x)[2] << endl;

            cv::cvtColor(srcImage, dstImage, cv::COLOR_BGR2GRAY);
            cout << "Gray: " << (int)dstImage.at<uchar>(y, x) << endl;
            cout << "x: " << x << '\t';
            cout << "y: " << y << endl;
        }
        break;
        }
    }
    void prosess_image()
    {
        std::vector<DETECT_RESULT> Detect_result;
        if (Req_detector == "yolov5")
        {
            Detect_result = Yolov5_detect.detect(Frame, Frameout);
            for (size_t i = 0; i < Detect_result.size(); i++)
            {
                if (Req_object == "all" || Req_object == Detect_result[i].name)
                {
                    // 在Scale_vec中寻找目标识别结果的变换尺度
                    for (size_t j = 0; j < Scale_vec.size(); j++)
                    {
                        if (Scale_vec[j].Name == Detect_result[i].name)
                        {
                            solve_pnp(Detect_result[i].Rect, Scale_vec[j].Wh);
                        }
                    }
                }
            }
        }
        else if (Req_detector == "opencv_color")
        {
            Detect_result = Color_detect.detect(Frame, Frameout, Req_object);
            for (size_t i = 0; i < Detect_result.size(); i++)
            {
                if (Req_object == "all" || Req_object == Detect_result[i].name)
                {
                    // 在Scale_vec中寻找目标识别结果的变换尺度
                    for (size_t j = 0; j < Scale_vec.size(); j++)
                    {
                        if (Scale_vec[j].Name == Detect_result[i].name)
                        {
                            solve_pnp(Detect_result[i].Rect, Scale_vec[j].Wh);
                        }
                    }
                }
            }
        }
        else if (Req_detector == "opencv_circle")
        {
            Detect_result = Circle_detect.detect(Frame, Frameout, Req_object);
            for (size_t i = 0; i < Detect_result.size(); i++)
            {
                if (Req_object == "all" || Req_object == Detect_result[i].name)
                {
                    // 在Scale_vec中寻找目标识别结果的变换尺度
                    for (size_t j = 0; j < Scale_vec.size(); j++)
                    {
                        if (Scale_vec[j].Name == Detect_result[i].name)
                        {
                            solve_pnp(Detect_result[i].Rect, Scale_vec[j].Wh);
                        }
                    }
                }
            }
            // 这里写霍夫圆检测，或者opencv颜色块识别
        }
        else
        {
            Frameout = Frame;
        }
        cv::namedWindow("Frameout");
        cv::imshow("Frameout", Frameout);
        setMouseCallback("Frameout", on_MouseHandle, &Frame);

        // cv:: imshow("frame",Frameout);

        int c = cv::waitKey(1); // 一般是1，相当于每秒1000张图片
    }

    bool image_server_callback(control_demo::image_srv::Request &req,
                               control_demo::image_srv::Response &res)
    {
        Req_detector = req.req_detector;
        Req_object = req.req_object;
        return true;
    }
    cv::Mat &Frame;
    cv::Mat Frameout;
    YOLOV5_DETECT Yolov5_detect;
    COLOR_DETECT Color_detect;
    CIRCLE_DETECT Circle_detect;
    string Req_detector = "opencv_circle";
    string Req_object = "circle1";
    std::vector<SCALE> Scale_vec;
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "image_server");
    ros::NodeHandle nh;
    ROS_INFO("opencv版本%s", CV_VERSION);
    // std::string gst_str = "rtspsrc location=rtsp://192.168.1.120:8554/test latency=10 ! rtph264depay ! avdec_h264 ! videorate ! videoconvert ! appsink sync=false";
    std::string gst_str = "v4l2src device=/dev/video0 ! video/x-raw, width=1280, height=720 ! videoconvert ! appsink";
    // std::string gst_str = "v4l2src device=/dev/video0 ! video/x-raw, width=640 height=640 ! videoconvert ! appsink";
    cv::VideoCapture cap(gst_str, cv::CAP_GSTREAMER);

    cv::Mat frame;
    IMAGE_PROSESS image_prosess(nh, frame);
    // ros::Rate loop_rate(20);
    if (cap.isOpened())
    {
        ROS_INFO("视频捕获成功");
        while (ros::ok())
        {
            auto start = std::chrono::system_clock::now();
            cap.read(frame);
            // cap.set(cv::CAP_PROP_FPS,10);
            // std::cout << "FPS: " << cap.get(cv::CAP_PROP_FPS) << std::endl;
            auto end = std::chrono::system_clock::now();
            // std::cout << "totletime: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
            if (frame.empty())
            {
                ROS_INFO("读取视频祯为空");
                break;
            }

            image_prosess.prosess_image();

            ros::spinOnce();
            // loop_rate.sleep();
        }
    }
    else
    {
        ROS_INFO("视频捕获失败");
    }
    cap.release();
    cv::destroyAllWindows();
    ros::Duration(20).sleep();
    return 0;
}
