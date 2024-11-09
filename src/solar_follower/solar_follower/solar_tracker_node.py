#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from rclpy.action import ActionClient
import math
from datetime import datetime, timedelta
import pytz

class SolarTracker(Node):
    def __init__(self):
        super().__init__('solar_tracker')
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.latitude = 40.4168
        self.longitude = -3.7038
        self.timezone = pytz.timezone('Europe/Madrid')
        self.start_time = datetime.now(self.timezone)
        
        # Actualizar cada 15 segundos
        self.create_timer(15.0, self.update_position)
        
        self.get_logger().info('Nodo de seguimiento solar iniciado')
        self.get_logger().info('Ubicación configurada: Madrid, España')
        self.get_logger().info(f'Latitud: {self.latitude}°, Longitud: {self.longitude}°')
        self.get_logger().info('Iniciando seguimiento solar...')
        
        # Primera actualización inmediata
        self.update_position()

    def calculate_sun_position(self, date):
        """Calcula la posición del sol para un momento dado del día."""
        hour = date.hour + date.minute/60.0 + date.second/3600.0
        
        # Para demostración, usamos un movimiento sinusoidal simplificado
        # que simula el movimiento del sol durante el día
        time_factor = (hour - 12) / 12  # -1 a 1 a lo largo del día
        
        # Calculamos el ángulo usando una función sinusoidal
        # El ángulo será máximo al mediodía y mínimo en la noche
        angle = -0.4 * math.sin(math.pi * time_factor)  # -0.4 a 0.4 radianes
        
        if 6 <= hour <= 18:  # Durante el día
            return angle
        else:  # Durante la noche
            return 0.0  # Panel en posición horizontal
        
    def update_position(self):
        current_time = datetime.now(self.timezone)
        
        # Simular 24 horas en 10 minutos (144 segundos = 1 hora simulada)
        minutes_since_start = (current_time - self.start_time).total_seconds() / 60.0
        simulated_hour = (minutes_since_start * 2.4) % 24  # 24 horas en 10 minutos
        
        demo_time = current_time.replace(
            hour=int(simulated_hour),
            minute=int((simulated_hour % 1) * 60)
        )
        
        target_angle = self.calculate_sun_position(demo_time)
        
        # Crear trayectoria
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['panel_joint']
        
        # Punto inicial
        point1 = JointTrajectoryPoint()
        point1.positions = [target_angle]
        point1.velocities = [0.0]
        point1.time_from_start = Duration(sec=0, nanosec=0)
        
        # Punto final
        point2 = JointTrajectoryPoint()
        point2.positions = [target_angle]
        point2.velocities = [0.0]
        point2.time_from_start = Duration(sec=5, nanosec=0)
        
        goal_msg.trajectory.points = [point1, point2]
        
        # Log más detallado
        status = "día" if 6 <= simulated_hour <= 18 else "noche"
        self.get_logger().info(
            f'Hora: {int(simulated_hour):02d}:{int((simulated_hour % 1) * 60):02d}, '
            f'Estado: {status}, '
            f'Ángulo panel: {math.degrees(target_angle):.2f}°'
        )
        
        # Enviar comando
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg)
        
def main(args=None):
    rclpy.init(args=args)
    solar_tracker = SolarTracker()
    try:
        rclpy.spin(solar_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        solar_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()