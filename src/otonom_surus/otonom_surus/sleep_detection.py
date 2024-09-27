#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 tabanlı göz kırpma ve uyku tespiti
@author: Erenn
"""

import cv2
import cvzone
import time
from cvzone.FaceMeshModule import FaceMeshDetector
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_srvs.srv import SetBool

class BlinkDetectorNode(Node):
    def __init__(self):
        super().__init__('blink_detector_node')

        # Publisher: ROS2'de sonuçları yayınlamak için
        self.publisher_ = self.create_publisher(Image, 'blink_image', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # Göz kırpma takibi değişkenleri
        self.detector = FaceMeshDetector(maxFaces=1)  # Sadece bir yüzü takip edelim
        self.eyeIdList = [159, 23, 130, 243]  # Göz noktaları listesi
        self.ratio_list = []
        self.eye_closed_start_time = 0
        self.is_eye_closed = False

        # Uyku durumu için servis
        self.sleep_srv = self.create_service(SetBool, 'check_sleep', self.handle_sleep_service)

        # Video yakalama
        self.cap = cv2.VideoCapture("/home/eren/Ern/new_beginners/otonom_ride_ws/src/otonom_surus/otonom_surus/videos/video.mp4")

        # Timer: Her 0.1 saniyede bir görüntüyü işler (FPS = 10)
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        """Her karede göz kırpma ve uyku durumunu işleyen ana fonksiyon."""
        success, img = self.cap.read()

        if not success:
            self.get_logger().info('Video akışı bitti.')
            return

        img, faces = self.detector.findFaceMesh(img, draw=False)

        if faces:
            face = faces[0]
            self._draw_eye_points(img, face)

            # Göz oranını hesaplayalım
            ratio = self._calculate_eye_aspect_ratio(face)
            self._update_eye_ratio_list(ratio)

            # Uyku tespiti
            if self._is_sleeping():
                self._handle_sleep(img)
            else:
                self._reset_awake_state(img)

        # Pencereye görüntü ekleyelim
        img = cv2.resize(img, (640, 360))  # Görüntüyü boyutlandır
        cv2.imshow("Blink Detection", img)
        cv2.waitKey(1)

        # Görüntüyü ROS2 mesajı olarak yayınla
        ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.publisher_.publish(ros_image)

    def _draw_eye_points(self, img, face):
        """Yüz üzerindeki göz noktalarını çizer."""
        for id in self.eyeIdList:
            cv2.circle(img, face[id], 3, (0, 0, 255), cv2.FILLED)

    def _calculate_eye_aspect_ratio(self, face):
        """Gözün dikey ve yatay uzunluklarını kullanarak göz oranını hesaplar."""
        left_up = face[159]
        left_down = face[23]
        left_left = face[130]
        left_right = face[243]

        # Gözün dikey ve yatay mesafelerini bulalım
        length_ver, _ = self.detector.findDistance(left_up, left_down)
        length_hor, _ = self.detector.findDistance(left_left, left_right)

        ratio = int((length_ver / length_hor) * 100)
        return ratio

    def _update_eye_ratio_list(self, ratio):
        """Göz oranı listesi güncellenir ve eski veriler atılır."""
        self.ratio_list.append(ratio)
        if len(self.ratio_list) > 3:
            self.ratio_list.pop(0)

    def _is_sleeping(self):
        """Göz oranına göre kişinin uyuyup uyumadığını kontrol eder."""
        ratio_avg = sum(self.ratio_list) / len(self.ratio_list)
        if ratio_avg < 33:  # Göz kapalıysa
            if not self.is_eye_closed:
                self.eye_closed_start_time = time.time()
                self.is_eye_closed = True
            return (time.time() - self.eye_closed_start_time) > 2
        else:
            self.is_eye_closed = False
            return False

    def _handle_sleep(self, img):
        """Uyku tespit edilirse yapılan işlemler."""
        cvzone.putTextRect(img, "Uyuyor!", (50, 150), colorR=(0, 0, 255), scale=2)
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(move_cmd)
        self.get_logger().error("Araç DURUYOR, uyku tespit edildi!")

    def _reset_awake_state(self, img):
        """Kişi uyumuyorsa yapılan işlemler."""
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # İleri hareket hızı
        move_cmd.angular.z = 0.0  # Düz gitmesi için
        self.cmd_vel_publisher.publish(move_cmd)

    def handle_sleep_service(self, request, response):
        """Uyku durumunu kontrol eden servis geri dönüşü."""
        if self.is_eye_closed and (time.time() - self.eye_closed_start_time) > 2:
            response.success = True
            response.message = "Uyku durumu aktif."
        else:
            response.success = False
            response.message = "Uyku durumu tespit edilmedi."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BlinkDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
