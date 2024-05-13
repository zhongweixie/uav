#! /usr/bin/env python3

import rospy
import cv2
import yolov5
import numpy as np

from offboard.srv import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import pyzbar.pyzbar as pyzbar

import platform
import pathlib
plt = platform.system()
if plt != 'Windows':
  pathlib.WindowsPath = pathlib.PosixPath

over = 0

dx = 0
dy = 0

num = 0
type = "None"
type_detect = "detect"
type_land = "land"

# 相机内参矩阵
camera_matrix = np.array([[368.583217,0.000000,639.513812],
                          [0.000000,368.650947,359.808990],
                          [0.000000,0.000000,1.000000]])
# 相机畸变系数
dist_coeffs = np.array([-0.000227, 0.000069, -0.000086, 0.000282, 0.000000])

import sys
yolov5_path = '/home/tzl/ego_v1_ws/src/offboard/wyj/yolov5-master'  # 将 '/path/to/yolov5' 替换为 YOLOv5 文件夹的实际路径
sys.path.append(yolov5_path)

import argparse
import csv
import os
import platform
import sys
from pathlib import Path

import torch

# FILE = Path(__file__).resolve()
# ROOT = FILE.parents[0]  # YOLOv5 root directory
# if str(ROOT) not in sys.path:
#     sys.path.append(str(ROOT))  # add ROOT to PATH
# ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from ultralytics.utils.plotting import Annotator, colors, save_one_box

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (
    LOGGER,
    Profile,
    check_file,
    check_img_size,
    check_imshow,
    check_requirements,
    colorstr,
    cv2,
    increment_path,
    non_max_suppression,
    print_args,
    scale_boxes,
    strip_optimizer,
    xyxy2xywh,
)
from utils.torch_utils import select_device, smart_inference_mode
from utils.augmentations import (
    Albumentations,
    augment_hsv,
    classify_albumentations,
    classify_transforms,
    copy_paste,
    letterbox,
    mixup,
    random_perspective,
)

# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device = 'cuda:0'
weights = '/home/tzl/ego_v1_ws/src/offboard/wyj/yolov5/best2.pt'  #ROOT / 'my_models/model1/best.pt'
dnn = False
data = "/home/tzl/ego_v1_ws/src/offboard/wyj/yolov5/my.yaml"
half = False
imgsz = [640,640]
source = '0'
max_det=1000 #每张图片最多检测的目标数，默认为1000
vid_stride=1 # video frame-rate stride 视频帧采样间隔，默认为1，即每一帧都进行检测
augment=False #推理的时候进行多尺度，翻转等操作(TTA)推理
visualize=False #visualize为是否可视化
conf_thres=0.25  # confidence threshold
iou_thres=0.45  # NMS IOU threshold
line_thickness=3
hide_conf=False
classes = None #检测的类别，默认为None，即检测所有类别，如果设置了该参数，则只检测该参数指定的类别
agnostic_nms=False #进行NMS去除不同类别之间的框，默认为False
auto = True
need_video = False #是否需要录制检测的视频
show_img = True
# Load model
device = select_device(device)
model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
stride, names, pt = model.stride, model.names, model.pt
imgsz = check_img_size(imgsz, s=stride)  # check image size
# Run inference
bs = 1
model.warmup(imgsz=(1 if pt or model.triton else 1, 3, *imgsz))  # warmup
seen, windows, dt = 0, [], (Profile(), Profile(), Profile())
# seen, windows, dt = 0, [], (Profile(device=device), Profile(device=device), Profile(device=device))


# classesFile = "/home/wyj/yolov5/my.names"
# classNames = []
# with open(classesFile, 'rt') as f:
#     classNames = f.read().rstrip('\n').split('\n')
# model = yolov5.load('/home/wyj/yolov5/best1.pt')# yolov5n6.pt这个文件是在github上下载的
# model.conf = 0.25  # NMS confidence threshold
# model.iou = 0.45  # NMS IoU threshold
# model.agnostic = False  # NMS class-agnostic
# model.multi_label = False  # NMS multiple labels per box
# model.max_det = 1000  # maximum number of detections per image

def do_client(request):
    global over
    global type
    global dx
    global dy
    global num
    num = request.num
    type = request.type
    print("请求类型为%s\n" %type)
    print("请求任务为%d\n" %num)
    response = xiaosai_detect_typeResponse()
    response.is_detect = "服务请求成功"
    # image_converter()
    # while over == 0:
    #     print("未检测结束")
    # if over == 1:
    #     response.is_detect = "服务请求成功"
    #     response.dx = dx 
    #     response.dy = dy
    #     print("已回调")
    return response

# def detector(img):
#     global over
#     global dx
#     global dy 
#     classIds = []

#     results = model(img, size = 1280 ,augment=True)
#     predictions = results.pred[0]
#     boxes = predictions[:, :4] # x1, y1, x2, y2
#     scores = predictions[:, 4]
#     categories = predictions[:, 5]
#     length = boxes.shape[0]
#     if length == 0:
#         print("no target")
#         return 
#     print(scores)
#     # img = cv2.UMat(img)
#     for i in range(length):
#         x1 = int(boxes[i][0])
#         y1 = int(boxes[i][1])
#         x2 = int(boxes[i][2])
#         y2 = int(boxes[i][3])
#         cv2.rectangle(img, (x1, y1), (x2,y2), (255, 0 , 255), 2)
#         classIds.append(int(categories[i]))
#         print(classNames[classIds[i]])
#         cv2.putText(img, f'{classNames[classIds[i]]} {int(scores[i]*100)}%',
#                     (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
#         # 获取类别标签
#         class_id = int(categories[i])
#         class_name = classNames[class_id]
#         if class_name == 'tanke':
#             rospy.set_param("dx",x1)
#             rospy.set_param("dy",y1)
#     rospy.set_param("is_detect",1)
#     #     dx = x1
#     #     dy = y1
#     # over = 1

def detector(frame):
    global num
    #seen, windows, dt = 0, [], (Profile(device=device), Profile(device=device), Profile(device=device))
    im = letterbox(frame, imgsz, stride=stride, auto=auto)[0]
    im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    im = np.ascontiguousarray(im)
    with dt[0]:  # 开始计时,读取图片
        im = torch.from_numpy(im).to(model.device)  # 将图片转换为tensor，并放到模型的设备上，pytorch模型的输入必须是tensor
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0#将图片归一化，将图片像素值从0-255转换为0-1
        if len(im.shape) == 3:  # 在前面添加batch维度，即将图片的维度从3维转换为4维，即(3,640,640)转换为(1,3,640,640)，pytorch模型的输入必须是4维的
            im = im[None]  # expand for batch dim
    # 这一步电脑前置摄像头会打开
    #dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
    # bs = len(dataset) #1 int类型
    # Inference
    with dt[1]:  # 开始计时,推理时间
        pred = model(im, augment=augment, visualize=visualize)
    with dt[2]:  # 开始计时,NMS时间
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        # Process predictions 处理预测结果
        # per image,遍历每张图片,enumerate()函数将pred转换为索引和值的形式，
        # i为索引，det为对应的元素，即每个物体的预测框
        s=''
    for i, det in enumerate(pred):  # per image
        #seen += 1  # 检测的图片数量加1
        im0 = frame.copy()
        s += "%gx%g " % im.shape[2:]  # print string 打印照片的宽和长
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        annotator = Annotator(im0, line_width=line_thickness, example=str(names))
        if len(det):  ##如果预测框的数量大于0
            # Rescale boxes from img_size to im0 size
            # 将预测框的坐标从归一化坐标转换为原始坐标,im.shape[2:]为图片的宽和高，
            # det[:, :4]为预测框的坐标，im0.shape为图片的宽和高
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
            # Print results
            for c in det[:, 5].unique():
                n = (det[:, 5] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                # Write results
                # 遍历每个预测框,xyxy为预测框的坐标，conf为置信度，cls为类别,
                # reversed()函数用于将列表反转，*是一个扩展语法，
                # *xyxy表示将xyxy中的元素分别赋值给x1,y1,x2,y2
            for *xyxy, conf, cls in reversed(det):
                c = int(cls)  # integer class
                label = names[c] if hide_conf else f"{names[c]}"
                confidence = float(conf)
                confidence_str = f"{confidence:.2f}"
                label = names[c] if hide_conf else f"{names[c]} {conf:.2f}"
                # 绘制预测框和标签
                annotator.box_label(xyxy, label, color=colors(c, True))
                rospy.loginfo(xyxy)
                my_label = names[c]
                if my_label == 'tanke' and num == 1:
                    rospy.loginfo(xyxy)
                    rospy.loginfo("进入了")
                    x1 = int(xyxy[0].item())
                    y1 = int(xyxy[1].item())
                    x2 = int(xyxy[2].item())
                    y2 = int(xyxy[3].item())
                    # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    image_points = np.array([[x1,y1],[x1,y2],[x2,y2],[x2,y1]],dtype=float)
                    model_points = np.array([[-0.5,-0.5,0.0],[-0.5,0.5,0.0],[0.5,0.5,0.0],[0.5,-0.5,0.0]],dtype=float) #单位m
                    _, rotation_vector, translation_vector = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_P3P)
                    x = float(translation_vector[0])
                    y = float(translation_vector[1])
                    print("Rotation Vector:\n", rotation_vector)
                    print("Translation Vector:\n", translation_vector)
                    rospy.set_param("dx",x)
                    rospy.set_param("dy",y)
                    my_label = "None"
                if my_label == 'car' and num == 2:
                    rospy.loginfo(xyxy)
                    rospy.loginfo("进入了")
                    x1 = int(xyxy[0].item())
                    y1 = int(xyxy[1].item())
                    x2 = int(xyxy[2].item())
                    y2 = int(xyxy[3].item())
                    # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    image_points = np.array([[x1,y1],[x1,y2],[x2,y2],[x2,y1]],dtype=float)
                    model_points = np.array([[-0.5,-0.5,0.0],[-0.5,0.5,0.0],[0.5,0.5,0.0],[0.5,-0.5,0.0]],dtype=float) #单位m
                    _, rotation_vector, translation_vector = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_P3P)
                    x = float(translation_vector[0])
                    y = float(translation_vector[1])
                    print("Rotation Vector:\n", rotation_vector)
                    print("Translation Vector:\n", translation_vector)
                    rospy.set_param("dx",x)
                    rospy.set_param("dy",y)
                    my_label = "None"
                if my_label == 'bridge' and num == 3:
                    rospy.loginfo(xyxy)
                    rospy.loginfo("进入了")
                    x1 = int(xyxy[0].item())
                    y1 = int(xyxy[1].item())
                    x2 = int(xyxy[2].item())
                    y2 = int(xyxy[3].item())
                    # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    image_points = np.array([[x1,y1],[x1,y2],[x2,y2],[x2,y1]],dtype=float)
                    model_points = np.array([[-0.5,-0.5,0.0],[-0.5,0.5,0.0],[0.5,0.5,0.0],[0.5,-0.5,0.0]],dtype=float) #单位m
                    _, rotation_vector, translation_vector = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_P3P)
                    x = float(translation_vector[0])
                    y = float(translation_vector[1])
                    print("Rotation Vector:\n", rotation_vector)
                    print("Translation Vector:\n", translation_vector)
                    rospy.set_param("dx",x)
                    rospy.set_param("dy",y)
                    my_label = "None"
                if my_label == 'land' and num == 4:
                    rospy.loginfo(xyxy)
                    rospy.loginfo("进入了")
                    x1 = int(xyxy[0].item())
                    y1 = int(xyxy[1].item())
                    x2 = int(xyxy[2].item())
                    y2 = int(xyxy[3].item())
                    # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    image_points = np.array([[x1,y1],[x1,y2],[x2,y2],[x2,y1]],dtype=float)
                    model_points = np.array([[-0.5,-0.5,0.0],[-0.5,0.5,0.0],[0.5,0.5,0.0],[0.5,-0.5,0.0]],dtype=float) #单位m
                    _, rotation_vector, translation_vector = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_P3P)
                    x = float(translation_vector[0])
                    y = float(translation_vector[1])
                    print("Rotation Vector:\n", rotation_vector)
                    print("Translation Vector:\n", translation_vector)
                    rospy.set_param("dx",x)
                    rospy.set_param("dy",y)
                    my_label = "None"
        im0 = annotator.result()  # 获取绘制预测框和标签的图片
        if(show_img): #如果要实时显示识别结果图
            cv2.imshow('detect_result', im0)
            # cv2.waitKey(1)
    # Print time (inference-only)
    LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")
    rospy.set_param("is_detect",1)
    
        
def lander(img):
    pass

class image_converter:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris_0/camera/image_raw", Image, self.callback)   #/iris_0/camera/image_raw

    def callback(self,data):
        global type
        # 使用cv_bridge将ROS的图像数据转换成opencv的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        if type == type_detect:
            detector(cv_image)
        if type == type_land:
            detector(cv_image)
        type = "None"

        # 显示opencv格式的图像
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyWindow('Image window')

        # 再将opencv格式的数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    # 创建识别服务的服务端
    rospy.init_node("xiaosai_image_conventer")
    server = rospy.Service("xiaosai_detect_task", xiaosai_detect_type, do_client)
    rospy.loginfo("服务器已经启动了")
    # 创建图像传输订阅者
    # rospy.Subscriber("/iris_0/camera/image_raw", Image, callback)
    image_converter()
    rospy.loginfo("open succeed")
    rospy.spin()