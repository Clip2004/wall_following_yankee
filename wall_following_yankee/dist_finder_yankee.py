#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class DistFinderNode(Node):
    def __init__(self):
        super().__init__("dist_finder_yankee")

        # Suscripción al LIDAR
        self.scan_sbs = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)

        # Publicación del error de distancia
        self.error_pub = self.create_publisher(Float32, '/error', 10)

        # Timer para calcular y publicar el error cada 0.5 s
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Inicialización de variables
        self.a = float('inf')
        self.b = float('inf')
        self.th = math.pi / 4  # 45 grados
        self.alpha = 0.0
        self.AB = 0.0
        self.CD = 0.0
        self.AC = 0.2 # 20 cm
        self.e = 0.0
        self.tray_d = 2.0 # Distancia deseada al muro

    def getRanges(self, msg, index_a, index_b):
        # Verifica índices válidos y guarda distancias
        if 0 <= index_a < len(msg.ranges):
            self.a = msg.ranges[index_a]
        else:
            self.a = float('inf')

        if 0 <= index_b < len(msg.ranges):
            self.b = msg.ranges[index_b]
        else:
            self.b = float('inf')

    def scan_callback(self, msg):

        # Actualiza valores a y b del LIDAR
        index_a = 360 - int((math.pi / 2 - msg.angle_min) / msg.angle_increment)
        index_b = 360 - int((math.pi / 2 - self.th - msg.angle_min) / msg.angle_increment)
        self.getRanges(msg, index_a, index_b)
        # Calcula error solo si los rangos son válidos

        if self.a == float('inf') or self.b == float('inf'):
            self.get_logger().warn("Rangos no válidos, esperando datos del LIDAR...")
            return

        self.alpha = math.atan2(self.a * math.cos(self.th) - self.b, self.a * math.sin(self.th))
        self.AB = self.b * math.cos(self.alpha)
        self.CD = self.AB + self.AC * math.sin(self.alpha)
        self.e = self.tray_d - self.CD

    def timer_callback(self):
    # Solo publica el error calculado en scan_callback
        error_msg = Float32()
        error_msg.data = self.e
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistFinderNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()