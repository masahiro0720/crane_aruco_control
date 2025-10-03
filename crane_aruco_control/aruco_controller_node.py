import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String # 制御コマンド用
from cv_bridge import CvBridge
import cv2
import numpy as np

# --- ARUCOノードのロジックをベースに、制御機能を追加 ---
class ArucoControllerNode(Node):
    def __init__(self):
        super().__init__("aruco_controller_node")
        
        # --- パラメータ設定 (aruco_nodeから流用) ---
        self.marker_size = 0.055 
        dictionary_id_name = "DICT_5X5_250"
        
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
        except AttributeError:
            self.get_logger().error(f"Invalid Aruco dictionary ID: {dictionary_id_name}")
            return
            
        # --- OpenCV APIの修正を適用 ---
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        
        self.bridge = CvBridge()
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # 【追加】最後に処理したマーカーIDを保持
        self.last_detected_id = None 

        # --- サブスクリプション ---
        # CameraInfoを受け取るまで、画像処理は中断されます
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.info_callback, rclpy.qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Image, '/custom_image_raw', self.image_callback, rclpy.qos.qos_profile_sensor_data
        )

        # --- パブリッシャ ---
        # 制御コマンドをCRANE+ドライバに送信
        self.command_pub = self.create_publisher(String, 'crane_command', 10)
        
        self.get_logger().info("Aruco Controller Node Initialized. Awaiting Camera Info...")

        # --- 制御マッピング ---
        # マーカーIDと CRANE+ 動作コマンドの対応付け
        self.command_map = {
            1: "RESET_ARM",     # ID 1: 初期姿勢に戻す (vertical)
            2: "HOME_POSE",     # ID 2: ホーム姿勢に戻す
            3: "OPEN_GRIPPER",  # ID 3: グリッパーを開く
            4: "CLOSE_GRIPPER", # ID 4: グリッパーを閉じる
            5: "WAVE_HAND",     # ID 5: 手を振る動作
        }

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # CameraInfo受信完了。これ以降は購読を停止
        self.destroy_subscription(self.info_sub)
        self.get_logger().info("Camera Info Received. Starting marker detection.")

    def image_callback(self, img_msg):
        if self.info_msg is None:
            # CameraInfoを受信していない場合、処理を中断
            return

        # 画像メッセージをOpenCV画像に変換 (モノクロで処理速度を優先)
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")

        # --- ARUCOマーカー検出 ---
        (corners, marker_ids, rejected) = self.detector.detectMarkers(cv_image)

        if marker_ids is not None:
            # 最も手前にある（または最初に検出された）マーカーを処理
            detected_id = marker_ids[0][0] # IDを取得
            
            # --- 制御ロジックの実行 ---
            self.execute_control_logic(detected_id)
            
            # 【追加】最後に検出されたIDを記録
            self.last_detected_id = detected_id

        else:
            # 【追加】マーカーが検出されなかった場合、最後に検出されたIDをリセット
            if self.last_detected_id is not None:
                self.get_logger().info(f"Marker {self.last_detected_id} lost. Resetting control lock.")
                self.last_detected_id = None


    def execute_control_logic(self, marker_id):
        # 【修正】最後に検出したIDと同じ場合は処理をスキップ
        if marker_id == self.last_detected_id:
            return
            
        # 検出されたIDに基づいてコマンドをパブリッシュ
        command = self.command_map.get(marker_id)
        
        if command:
            msg = String()
            msg.data = command
            self.command_pub.publish(msg)
            self.get_logger().info(f"Detected ID {marker_id}. Publishing command: {command}")
        else:
            # 既知のIDが検出されなかった場合
            self.get_logger().warn(f"Detected unknown ID: {marker_id}. No command published.")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
