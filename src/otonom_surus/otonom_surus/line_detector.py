import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectionNode(Node):

    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Kamera verisini /camera/image_raw topic'inden alıyoruz.
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Hareket komutları için karar verici düğüme mesaj yayımlıyoruz
        self.cmd_vel_publisher = self.create_publisher(Twist, '/lane_follow/cmd_vel', 10)

        # OpenCV'ye görüntü dönüştürmek için CvBridge kullanıyoruz.
        self.bridge = CvBridge()

        # Çizgilerin kaybolmasını önlemek için son tespit edilen çizgileri saklıyoruz
        self.last_left_line = None
        self.last_right_line = None

        # Giriş ve çıkış logları
        self.get_logger().info('Lane Detection Node Başlatıldı.')

    def region_of_interest(self, image, vertices):
        """İlgi alanı dışındaki bölgeleri maskele"""
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, vertices, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def draw_lines(self, image, lines):
        """Tespit edilen çizgileri görüntü üzerine çiz"""
        image = np.copy(image)
        blank_image = np.zeros_like(image)

        left_line, right_line = None, None
        mid_point = None

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=10)

                    # Sol ve sağ çizgileri tespit et (görüntü merkezine göre)
                    if x1 < image.shape[1] // 2 and x2 < image.shape[1] // 2:
                        left_line = (x1, y1, x2, y2)
                    elif x1 > image.shape[1] // 2 and x2 > image.shape[1] // 2:
                        right_line = (x1, y1, x2, y2)

        # Kaybolan çizgilerde son tespit edilen çizgiyi kullan
        left_line = left_line or self.last_left_line
        right_line = right_line or self.last_right_line

        if left_line and right_line:
            left_mid = ((left_line[0] + left_line[2]) // 2, (left_line[1] + left_line[3]) // 2)
            right_mid = ((right_line[0] + right_line[2]) // 2, (right_line[1] + right_line[3]) // 2)
            mid_point = ((left_mid[0] + right_mid[0]) // 2, (left_mid[1] + right_mid[1]) // 2)
            
            # Orta noktayı çiz ve harekete karar ver
            cv2.circle(blank_image, mid_point, 10, (0, 0, 255), -1)
            self.decide_movement(mid_point[0], image.shape[1] // 2)
            
            # Sol ve sağ çizgi ortasına da bir çizgi ekle
            cv2.line(blank_image, left_mid, right_mid, (255, 0, 0), thickness=5)

        # Çizgilerin kaybolmasını önlemek için son tespit edilen çizgileri güncelle
        self.last_left_line = left_line
        self.last_right_line = right_line

        return cv2.addWeighted(image, 0.8, blank_image, 1, 0.0)

    def decide_movement(self, mid_x, image_center_x):
        """Robotun hareket kararını al"""
        move_cmd = Twist()

        if mid_x < image_center_x - 45:
            move_cmd.linear.x = 0.1  # İleri git
            move_cmd.angular.z = 0.3  # Sağa dön
            self.get_logger().info(f"Sağa dönülüyor: mid_x={mid_x}, center_x={image_center_x}")
        
        elif mid_x > image_center_x + 45:
            move_cmd.linear.x = 0.1  # İleri git
            move_cmd.angular.z = -0.3  # Sola dön
            self.get_logger().info(f"Sola dönülüyor: mid_x={mid_x}, center_x={image_center_x}")
        
        else:
            move_cmd.linear.x = 0.2  # İleri git
            move_cmd.angular.z = 0.0  # Düz git
            self.get_logger().info(f"Düz gidiliyor: mid_x={mid_x}, center_x={image_center_x}")

        self.cmd_vel_publisher.publish(move_cmd)

    def process(self, image):
        """Görüntü işleme: Canny edge ve Hough çizgi tespiti"""
        height, width = image.shape[0], image.shape[1]

        # İlgi alanı bölgesi
        region_of_interest_vertices = [(0, height), (0, height * 0.64), (width, height * 0.64), (width, height)]

        # Gri tonlamaya çevir ve kenarları tespit et
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_canny = cv2.Canny(img_gray, 250, 120)

        # İlgi alanı dışını maskeyle
        img_cropped = self.region_of_interest(img_canny, np.array([region_of_interest_vertices], np.int32))

        # Hough çizgi tespiti
        lines = cv2.HoughLinesP(img_cropped, rho=2, theta=np.pi / 180, threshold=200, lines=np.array([]), minLineLength=150, maxLineGap=18)

        return self.draw_lines(image, lines) if lines is not None else image

    def image_callback(self, msg):
        """Görüntü geldiğinde işleme ve gösterme"""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        processed_frame = self.process(frame)

        # Görüntüyü yeniden boyutlandır ve göster
        resized_frame = cv2.resize(processed_frame, (640, 480), interpolation=cv2.INTER_AREA)
        cv2.imshow("Processed Image", resized_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()

    try:
        rclpy.spin(lane_detection_node)
    except KeyboardInterrupt:
        pass
    
    lane_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
