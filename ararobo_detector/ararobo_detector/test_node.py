import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import datetime
import os

class RealsenseCamera:
    def __init__(self):
        self.WIDTH = 640#幅640ピクセル
        self.HEIGHT = 480#高さ480ピクセル
        self.FPS = 30#フレームレート30FPS
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, self.FPS)#カラー画像、BGR形式、ストリームを有効化
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.FPS)#深層画像、16bit、ストリームを有効化
    def start(self):#カメラのストリーミングを開始
        
        self.pipeline = rs.pipeline()
        self.pipeline.start(self.config)
        print("start")
    def read(self,is_array = True):#カメラからカラー画像と深層画像を取得
        ret = True
        frames = self.pipeline.wait_for_frames()#新しいフレームを取得
        self.color_frame = frames.get_color_frame()#カラー画像を取得
        self.depth_frame = frames.get_depth_frame()#深層画像を取得
        if not self.color_frame or not self.depth_frame:#カラー画像もしくは深層画像が習得できなかったらFalse
            ret = False
            return ret, (None, None)
        elif is_array:#カラー画像と深層画像をnumpy配列に変換
            color_image = np.array(self.color_frame.get_data())
            depth_image = np.array(self.depth_frame.get_data())
            return ret,(color_image,depth_image)
        else:
            return ret,(self.color_frame,self.depth_frame)
        
    def release(self):#カメラのストリーミングを停止
        self.pipeline.stop()

class test_node(Node):
    def changed_SV(self, bgr_img,alpha,beta,color_idx):
        hsvimage = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV_FULL)
        hsvf = hsvimage.astype(np.float32)
        hsvf[:,:,color_idx] = np.clip(hsvf[:,:,1]*alpha+beta,0,255)
        hsv8 = hsvf.astype(np.uint8)
        return cv2.cvtColor(hsv8,cv2.COLOR_HSV2BGR_FULL)
    
    def changed_S(bgr_img,alpha,beta):
        return changed_SV(bgr_img,alpha,beta,1)

#    def timerCb(self):
#       ret, frame = self.cap.read()
#       if ret > 0:
#           image, rects = self.detectCan(frame) # 缶を検出
#           if "detected" in self.debug_output:
#               cv2.imshow("detected image", image)
#               cv2.waitKey(1)
#           angle_msg = UInt16MultiArray()
#           angle_msg.data = self.objectsFilter(rects)
#           self.pub_can_angle.publish(angle_msg)
#           if self.log_image:
#               self.log_image_writer.write(image)
#           if self.image_pub:
#               image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
#               self.pub_can_image.publish(image_msg)
#
#    def detectCan (self, raw_image):
#        image = cv2.GaussianBlur(raw_image,(self.blur_ksize,self.blur_ksize),0)
#        if "blur" in self.debug_output:
#            cv2.imshow("blur", image)
#        hsv_image =cv2.cvtColor(image,cv2.COLOR_BGR2HSV_FULL)
#        if self.team_color:
#            hsv_mask_image = cv2.inRange(hsv_image,tuple(self.hsv_red_1[0]),tuple(self.hsv_red_1[1]))
#            hsv_mask_image |= cv2.inRange(hsv_image,tuple(self.hsv_red_2[0]),tuple(self.hsv_red_2[1]))
#        else:
#            hsv_mask_image = cv2.inRange(hsv_image,tuple(self.hsv_blue[0]),tuple(self.hsv_blue[1]))
#        if "hsv" in self.debug_output:
#            cv2.imshow("HSV_range", hsv_mask_image)
#        opening_image = cv2.morphologyEx(hsv_mask_image, cv2.MORPH_CLOSE, self.opening_ksize,iterations=5)
#        if "opening" in self.debug_output:
#            cv2.imshow("opening", opening_image)
#        contours,hierarchy = cv2.findContours(opening_image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
#        if "contours" in self.debug_output:
#            overlay_image = raw_image.copy()
#            for c in contours
#                cv2.polylines(overlay_image,c,True (255,0,0),2,cv2.LINE_AA)
#            cv2.imshow("contours_image", overlay_image)
#        approxes = []
#        for c in contours:
#            area = cv2.contourArea(c)
#            if area < self.contour_area_min:
#                continue

    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Hello World!')
        cap = RealsenseCamera()
        cap.WIDTH = 640
        cap.HEIGHT = 480
        cap.FPS = 30
        

        cap.start()

        while True:
            ret,frames = cap.read()
            color_frame = frames[0]
            depth_frame = frames[1]

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.08), cv2.COLORMAP_JET)

            images = np.hstack((color_frame, depth_colormap))
            cv2.imshow('RealSense', images)
            # bgr_omg = cv2.imread("RealSense",cv2.IMREAD_COLOR)
            images_SVH = self.changed_SV(images,2.5,0,1)
            cv2.imshow('RealSense_SV', images_SVH)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)
    node = test_node()
    rclpy.spin(node)
    rclpy.shutdown()
