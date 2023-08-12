#!/usr/bin/env python
import cv2
import os

#图片路径
im_dir = '/home/jetson/output-video/20230801'
#输出视频路径
video_dir = '/home/suanfa/data/out/201708231503440-1018.avi'
#帧率
fps = 30  
#图片数 
num = 426
#图片尺寸
img_size = (841,1023)


fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
videoWriter = cv2.VideoWriter(video_dir, fourcc, fps, img_size)

for i in range(1,num):
    
    videoWriter.write(frame)
    print im_name

videoWriter.release()
print("Finish")