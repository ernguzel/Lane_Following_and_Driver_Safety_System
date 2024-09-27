#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Otonom sürüş ve uyku tespiti için karar veren düğüm
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool  # Uyku tespiti servisi

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # Otonom sürüş komutlarını dinleme
        self.subscription = self.create_subscription(
            Twist, 
            '/lane_follow/cmd_vel', 
            self.twist_callback, 
            10)

        # Karar verilen cmd_vel için publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Uyku durumunu sorgulamak için servis istemcisi
        self.cli = self.create_client(SetBool, 'check_sleep')
        self.isSleeping = False  # Uyku durumu

        # Uyku servisine erişim kontrolü
        self._wait_for_sleep_service()

    def _wait_for_sleep_service(self):
        """Uyku tespit servisine bağlanmaya çalış."""
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Uyku servisi bekleniyor...')
        self.get_logger().info('Uyku servisine bağlanıldı.')

    def check_sleep_state(self):
        """Uyku durumu sorgulaması yap."""
        request = SetBool.Request()
        future = self.cli.call_async(request)
        future.add_done_callback(self.sleep_callback)

    def sleep_callback(self, future):
        """Uyku durumu servisi çağrısı sonrası geri dönüş."""
        try:
            response = future.result()
            if response.success != self.isSleeping:
                self.isSleeping = response.success
                self.get_logger().info(f"Uyku durumu değişti: {self.isSleeping}")
        except Exception as e:
            self.get_logger().error(f"Uyku durumu sorgusu başarısız: {e}")

    def twist_callback(self, msg):
        """Otonom sürüş komutlarını al ve uyku durumuna göre karar ver."""
        # Uyku durumunu her hareket komutu geldiğinde sorgula
        self.check_sleep_state()

        # Eğer uyku durumu tespit edildiyse hareketi durdur
        if self.isSleeping:
            self._publish_stop_cmd()
        else:
            # Uyku yoksa gelen komutları ilet
            self.cmd_vel_publisher.publish(msg)
            self.get_logger().info("Otonom sürüş: Komut iletiliyor.")

    def _publish_stop_cmd(self):
        """Robotu durdurmak için komut gönder."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_cmd)
        self.get_logger().info("Uyku modu: Robot durduruldu.")

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Düğüm sonlandırılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
