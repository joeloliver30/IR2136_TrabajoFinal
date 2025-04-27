import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import UInt16
from geometry_msgs.msg import PoseArray
import time

def arm_vehicle(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("🚀 Drone armado.")

def takeoff(connection, altitude):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    print(f"🛫 Despegando a {altitude} metros")

def set_mode(connection, mode):
    connection.set_mode(mode)
    print(f"🎛️ Modo cambiado a {mode}")

class BatteryGPSNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')

        # Conexión MAVLink
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        print("✅ Conectado a SITL")

        # Último valor de batería conocido
        self.last_battery_value = None

        # Battery publisher
        self.battery_publisher = self.create_publisher(UInt16, '/battery_status', 10)
        self.timer = self.create_timer(1.0, self.publish_battery)

        # Battery subscriber
        self.battery_subscriber = self.create_subscription(
            UInt16,
            '/battery_status',
            self.battery_callback,
            10
        )

        # Trajectory subscriber
        self.trajectory_subscriber = self.create_subscription(
            PoseArray,
            '/drone/gps',
            self.trajectory_callback,
            10
        )

        self.trajectory = []
        self.current_index = 0
        self.is_executing = False
        self.has_taken_off = False

    def battery_callback(self, msg):
        self.last_battery_value = msg.data
        print(f"🔄 (Backup) Batería actualizada desde ROS2: {self.last_battery_value}%")

    def publish_battery(self):
        msg = None
        start_time = time.time()
        while (time.time() - start_time) < 1.0:
            msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=False)
            if msg:
                break
        if msg:
            remaining = msg.battery_remaining
            pub = UInt16()
            pub.data = int(remaining)
            self.battery_publisher.publish(pub)
            print(f"🔋 Batería publicada: {remaining}%")

    def land_vehicle(self):
        print("🛬 Iniciando aterrizaje...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("🛬 Comando de aterrizaje enviado.")
        self.is_executing = False

    def trajectory_callback(self, msg):
        print(f"📦 Recibida trayectoria con {len(msg.poses)} puntos.")
        self.trajectory = msg.poses
        self.current_index = 0
        self.is_executing = True

        set_mode(self.connection, 'GUIDED')
        arm_vehicle(self.connection)
        takeoff(self.connection, 10)
        time.sleep(10)

        self.execute_next_point()

    def execute_next_point(self):
        if not self.is_executing:
            print("🚫 Ejecución interrumpida, no se continúa con la trayectoria.")
            return

        if self.current_index >= len(self.trajectory):
            print("✅ Trayectoria completada.")
            self.is_executing = False
            return

        point = self.trajectory[self.current_index]
        lat = point.position.x
        lon = point.position.y
        alt = point.position.z

        print(f"➡️ Navegando al punto {self.current_index + 1}/{len(self.trajectory)}")

        self.connection.mav.set_position_target_global_int_send(
            0,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0,  # aceleraciones
            0, 0
        )

        self.wait_until_arrival(lat, lon, alt)

    def wait_until_arrival(self, target_lat, target_lon, target_alt):
        while rclpy.ok() and self.is_executing:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
                current_alt = msg.relative_alt / 1000

                distance = ((target_lat - current_lat) ** 2 + (target_lon - current_lon) ** 2) ** 0.5
                alt_diff = abs(target_alt - current_alt)

                current_battery = self.read_battery_status() or self.last_battery_value

                print(f"📍 Lat: {current_lat:.6f}, Lon: {current_lon:.6f}, Alt: {current_alt:.2f}")
                print(f"🎯 Distancia: {distance:.6f}, ΔAlt: {alt_diff:.2f}")
                print(f"🔋 Batería actual: {current_battery}%")

                if current_battery <= 40:
                    print("⚠️ Batería baja detectada. Aterrizando inmediatamente...")
                    self.land_vehicle()
                    return

                if distance < 0.00001 and alt_diff < 0.5:
                    print("✅ Objetivo alcanzado")
                    self.current_index += 1
                    self.execute_next_point()
                    break

    def read_battery_status(self):
        msg = None
        start_time = time.time()
        while (time.time() - start_time) < 1.0:
            msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=False)
            if msg:
                return msg.battery_remaining
        return None

def main(args=None):
    rclpy.init(args=args)
    node = BatteryGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
