# camera_publisher_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo # <-- CameraInfoを追加
from cv_bridge import CvBridge
import cv2
import time
import numpy as np # <-- numpyを追加

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # パブリッシャの作成
        self.publisher_ = self.create_publisher(Image, 'custom_image_raw', 10)
        # CameraInfo パブリッシャの追加
        self.info_publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10) # <-- 追加
        
        # カメラデバイスの初期化 (0は通常内蔵またはデフォルトのUSBカメラ)
        self.cap = cv2.VideoCapture(0)
        
        # フレームレートを設定 (例: 30 FPS)
        timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        
        # ダミーの CameraInfo メッセージを作成 (暫定的なポーズ計算用)
        self.camera_info_msg = self._get_dummy_camera_info() # <-- 追加
        
        self.get_logger().info('Camera Publisher Node has started and is publishing to /custom_image_raw and /camera_info')

    def _get_dummy_camera_info(self):
        """ダミーの CameraInfo メッセージを作成する (キャリブレーションデータがない場合)"""
        info = CameraInfo()
        
        # ⚠️ 注意: 正確な値ではありません。動作確認用です。
        # 実際のポーズ推定にはカメラキャリブレーションが必要です。
        info.header.frame_id = 'camera_link'
        info.height = 480 # 画像のheightと一致させる
        info.width = 640  # 画像のwidthと一致させる
        
        # K (Intrinsic Camera Matrix): [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        # ポーズ計算を有効にするため、適当な焦点距離を設定
        fx = 500.0
        fy = 500.0
        cx = info.width / 2.0
        cy = info.height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        # D (Distortion Coefficients): [k1, k2, t1, t2, k3, ...]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        return info # <-- CameraInfoメッセージを返す

    def timer_callback(self):
        # カメラからフレームを読み込み
        ret, frame = self.cap.read()
        
        if ret:
            # OpenCV画像をROS Imageメッセージに変換
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # タイムスタンプを設定 (重要)
            current_time = self.get_clock().now().to_msg()
            img_msg.header.stamp = current_time
            img_msg.header.frame_id = 'camera_link' # 座標系のフレームIDを設定

            # CameraInfo メッセージを更新してパブリッシュ
            self.camera_info_msg.header.stamp = current_time
            self.camera_info_msg.header.frame_id = 'camera_link'
            
            # 画像とCameraInfoをパブリッシュ
            self.publisher_.publish(img_msg)
            self.info_publisher_.publish(self.camera_info_msg) # <-- CameraInfoをパブリッシュ
        else:
            self.get_logger().error('Failed to capture image from camera.')

    def destroy_node(self):
        # ノード終了時にカメラを解放
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
