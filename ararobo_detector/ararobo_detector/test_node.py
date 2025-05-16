import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs

class test_node(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Hello World!')

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

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()




def main(args=None):
    rclpy.init(args=args)
    node = test_node()
    rclpy.spin(node)
    rclpy.shutdown()