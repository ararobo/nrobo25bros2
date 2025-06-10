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

#class RealsenseCamera:
#    def __init__(self):
#        self.WIDTH = 640#幅640ピクセル
#        self.HEIGHT = 480#高さ480ピクセル
#        self.FPS = 30#フレームレート30FPS
#        self.config = rs.config()
#        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, self.FPS)#カラー画像、BGR形式、ストリームを有効化
#        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.FPS)#深層画像、16bit、ストリームを有効化
#    def start(self):#カメラのストリーミングを開始
#        
#        self.pipeline = rs.pipeline()
#        self.pipeline.start(self.config)
#        print("start")
#    def read(self,is_array = True):#カメラからカラー画像と深層画像を取得
#        ret = True
#        frames = self.pipeline.wait_for_frames()#新しいフレームを取得
#        self.color_frame = frames.get_color_frame()#カラー画像を取得
#        self.depth_frame = frames.get_depth_frame()#深層画像を取得
#        if not self.color_frame or not self.depth_frame:#カラー画像もしくは深層画像が習得できなかったらFalse
#            ret = False
#            return ret, (None, None)
#        elif is_array:#カラー画像と深層画像をnumpy配列に変換
#            color_image = np.array(self.color_frame.get_data())
#            depth_image = np.array(self.depth_frame.get_data())
#            return ret,(color_image,depth_image)        super().__init__('test_node')
#        self.get_logger().info('Hello World!')
#        cap = RealsenseCamera()
#        cap.WIDTH = 640
#        cap.HEIGHT = 480
#        cap.FPS = 30
    
#        else:
#            return ret,(self.color_frame,self.depth_frame)
#        
#    def release(self):#カメラのストリーミングを停止
#        self.pipeline.stop()

class test_node(Node):
    #SVの変更
#    def changed_SV(self, bgr_img,alpha,beta,color_idx):
#        hsvimage = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV_FULL)
#        hsvf = hsvimage.astype(np.float32)
#        hsvf[:,:,color_idx] = np.clip(hsvf[:,:,1]*alpha+beta,0,255)
#        hsv8 = hsvf.astype(np.uint8)
#        return cv2.cvtColor(hsv8,cv2.COLOR_HSV2BGR_FULL)
#    
#    def changed_S(bgr_img,alpha,beta):
#        return changed_SV(bgr_img,alpha,beta,1)
    #缶の検出
    fps = 30 # フレームレート
    image_width = 640 # 画像の幅
    image_height = 480 # 画像の高さ
    blur_ksize = 9 # ぼかしの度合い
    opening_ksize = (3,3)#ノイズ除去の度合い
    debug_output = ["detected"]
    hsv_red_1 = [[216,138,91],[280,262,261]]
    hsv_red_2 = [[0,138,91],[20,262,261]]
    hsv_blue = [[100,64,0],[300,255,255]]
    contour_area_min = 3000 # 検出する輪郭の最小面積
    y_range = (200,400)
    team_color = False
    inclination = 15
    image_pub = True
    log_image = False
    save_name = ""
    
##缶の検出
    def __init__(self):
        super().__init__("can_detector")
        #パラメーター
        self.hsv_red_1[0] = self.declare_parameter("hsv.red_1.min",self.hsv_red_1[0]).get_parameter_value().integer_array_value
        self.hsv_red_1[1] = self.declare_parameter("hsv.red_1.max",self.hsv_red_1[1]).get_parameter_value().integer_array_value
        self.hsv_red_2[0] = self.declare_parameter("hsv.red_2.min",self.hsv_red_2[0]).get_parameter_value().integer_array_value
        self.hsv_red_2[1] = self.declare_parameter("hsv.red_2.max",self.hsv_red_2[1]).get_parameter_value().integer_array_value
        self.hsv_blue[0] = self.declare_parameter("hsv.blue.min",self.hsv_blue[0]).get_parameter_value().integer_array_value
        self.hsv_blue[1] = self.declare_parameter("hsv.blue.max",self.hsv_blue[1]).get_parameter_value().integer_array_value
        self.debug_output = self.declare_parameter("debug_output", self.debug_output).get_parameter_value().string_array_value
        self.y_range = self.declare_parameter("y_range",self.y_range).get_parameter_value().integer_array_value
        self.contour_area_min = self.declare_parameter("contour_area_min",self.contour_area_min).get_parameter_value().integer_value
        self.team_color = self.declare_parameter("team_color",self.team_color).get_parameter_value().bool_value
        self.inclination = self.declare_parameter("inclination",self.inclination).get_parameter_value().integer_value
        self.image_pub = self.declare_parameter("image_pub",self.image_pub).get_parameter_value().bool_value
        self.log_image = self.declare_parameter("log_image",self.log_image).get_parameter_value().bool_value
        self.save_name = self.declare_parameter("save_name",self.save_name).get_parameter_value().string_value
        self.pub_can_angle = self.create_publisher(UInt16MultiArray,"can/pixls", 5)
        self.pub_can_image = self.create_publisher(Image,"can/image", 5)
        self.cap = cv2.VideoCapture(6)
        self.bridge = CvBridge()

        if self.log_image:
            self.iitimageLogger(self.save_name,True)
        self.timer = self.create_timer(0.033,self.timerCb) # 30FPSでコールバックを呼び出す


    def timerCb(self):
       ret, frame = self.cap.read()
       if ret > 0:
           image, rects = self.detectCan(frame) # 缶を検出
           if "detected" in self.debug_output:
               cv2.imshow("detected image", image)
               cv2.waitKey(1)
           angle_msg = UInt16MultiArray()
           angle_msg.data = self.objectsFilter(rects)
           self.pub_can_angle.publish(angle_msg)
           if self.log_image:
               self.log_image_writer.write(image)
           if self.image_pub:
               image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
               self.pub_can_image.publish(image_msg)

    def detectCan (self, raw_image):
        image = cv2.GaussianBlur(raw_image,(self.blur_ksize,self.blur_ksize),0)
        if "blur" in self.debug_output:
            cv2.imshow("blur", image)
        hsv_image =cv2.cvtColor(image,cv2.COLOR_BGR2HSV_FULL)
        if self.team_color:
            hsv_mask_image = cv2.inRange(hsv_image,tuple(self.hsv_red_1[0]),tuple(self.hsv_red_1[1]))
            hsv_mask_image |= cv2.inRange(hsv_image,tuple(self.hsv_red_2[0]),tuple(self.hsv_red_2[1]))
        else:
            hsv_mask_image = cv2.inRange(hsv_image,tuple(self.hsv_blue[0]),tuple(self.hsv_blue[1]))
        if "hsv" in self.debug_output:
            cv2.imshow("HSV_range", hsv_mask_image)
        opening_image = cv2.morphologyEx(hsv_mask_image, cv2.MORPH_CLOSE, self.opening_ksize,iterations=5)
        if "opening" in self.debug_output:
            cv2.imshow("opening", opening_image)
        contours,hierarchy = cv2.findContours(opening_image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        if "contours" in self.debug_output:
            overlay_image = raw_image.copy()
            for c in approxes:
                cv2.polylines(overlay_image,c,True (255,0,0),2,cv2.LINE_AA)
            cv2.imshow("contours_image", overlay_image)
        approxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.contour_area_min:
                continue
            approxes.append(cv2.convexHull(c))
        if "convex" in self.debug_output:
            overlay_image = raw_image.copy()
            for a in approxes:
                cv2.polylines(overlay_image,a,True (255,0,0),2,cv2.LINE_AA)
            cv2.imshow("convex", overlay_image)
        rects = []
        for a in approxes:
            rects.append(cv2.minAreaRect(a))
        for r in rects:
            box = cv2.boxPoints(r)
            box = np.int0(box)
            cv2.drawContours(raw_image,[box],0,(255,0,0),2)
        if "rect" in self.debug_output:
            cv2.imshow("rect", raw_image)
        return raw_image, rects
    
    def objectsFilter(self,rects):
        out = [0]
        for rect in rects:
            self.get_logger().info(f"y:{rect[0][1]}")
            if self.y_range[0] < rect[0][1] < self.y_range[1]:
                if rect[1][0] < rect[1][1]:
                    if rect[2] < self.inclination:
                        self.get_logger().info("standing")
                        out.append(int(rect[0][0]))
                    else:
                        self.get_logger().info("falling down")
                else:
                    if rect[2] > (90-self.inclination):
                        self.get_logger().info("standing")
                        out.append(int(rect[0][0]))
                        out[0] += 1
                    else:
                        self.get_logger().info("falling down")
        return out
    
    def initImageLogger(self,save_name,is_color):
        img_format = cv2.VideoWriter_fourcc('m','p','4','v')
        now = datetime.datetime.now()
        home_dir = os.path.expanduser("~")
        save_path = os.path.join(home_dir,'can_detector',str(now.day),save_name + str(now.hour) + "_" + str(now.minute) + '.mp4')
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        self.log_image_writer = cv2.VideoWriter(save_path,img_format,self.fps,(self.image_width,self.image_height),isColor=is_color)        

    def __del__(self):
        self.cap.release()
        self.log_image_writer.release()
##ここまで

#    def __init__(self):
#        super().__init__('test_node')
#        self.get_logger().info('Hello World!')
#        cap = RealsenseCamera()
#        cap.WIDTH = 640
#        cap.HEIGHT = 480
#        cap.FPS = 30
#        self.cap.release()
#        self.log_image_writer.release()
        

#        cap.start()
#
#        while True:
#            ret,frames = cap.read()
#            color_frame = frames[0]
#            depth_frame = frames[1]
#
#            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.08), cv2.COLORMAP_JET)
#
#            images = np.hstack((color_frame, depth_colormap))
#            cv2.imshow('RealSense', images)
#            # bgr_omg = cv2.imread("RealSense",cv2.IMREAD_COLOR)
#            images_SVH = self.changed_SV(images,2.5,0,1)
#            cv2.imshow('RealSense_SV', images_SVH)
#
#            if cv2.waitKey(1) & 0xFF == ord('q'):
#                break
#
#        cap.release()
#        cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)
    node = test_node()
    rclpy.spin(node)
    rclpy.shutdown()
