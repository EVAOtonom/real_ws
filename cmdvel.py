#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime
import os

class CmdVelLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_logger')
        
        # /cmd_vel topiğine abone oluyoruz
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Kayıt dosyasını 'append' (ekleme) modunda açıyoruz
        self.file_name = 'cmd_vel_log.txt'
        self.file = open(self.file_name, 'a')
        
        # Dosyanın başına bir başlık ekleyelim (eğer dosya yeniyse)
        if os.path.getsize(self.file_name) == 0:
            self.file.write("Tarih_Saat\t\t\tLinear_X\tAngular_Z\n")
            self.file.write("-" * 50 + "\n")
            self.file.flush()

        self.get_logger().info(f'/cmd_vel verileri {self.file_name} dosyasına kaydediliyor...')

    def cmd_vel_callback(self, msg: Twist):
        # O anki zamanı alıyoruz (Milisaniye hassasiyetinde)
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        # Gelen hız verileri
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Dosyaya yazılacak formatlı metin
        log_str = f"[{now}] \t {linear_x:+.4f} \t\t {angular_z:+.4f}\n"
        
        # Dosyaya yaz ve RAM'de beklemeden anında diske kaydet (flush)
        self.file.write(log_str)
        self.file.flush()
        
    def destroy_node(self):
        # Düğüm kapatılırken dosyayı güvenli bir şekilde kapat
        if not self.file.closed:
            self.file.close()
            self.get_logger().info('Kayıt dosyası kapatıldı.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C ile çıkıldığında sessizce kapan
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()