import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import serial

class ScaleVehicleNode(Node):
    def __init__(self):
        super().__init__('scale_vehicle_node')
        
        # Serial 연결
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Publishers
        self.steering_angle_pub = self.create_publisher(
            Float32, 
            'steering_angle_feedback', 
            10
        )
        self.speed_pub = self.create_publisher(
            Float32, 
            'speed_feedback', 
            10
        )
        
        # Subscribers
        self.steering_sub = self.create_subscription(
            Float32,
            'steering_cmd',
            self.steering_callback,
            10
        )
        self.speed_sub = self.create_subscription(
            Float32, 
            'speed_cmd',
            self.speed_callback,
            10
        )
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('steering_offset', 518),
                ('steering_range', 380),
                ('max_speed', 250)
            ]
        )
        
        # Current values
        self.current_speed = 0
        self.current_steering = 518  # 중립값
        self.current_gear = 0  # 0: 전진, 1: 중립, 2: 후진
        
        # Timer for reading feedback
        self.create_timer(0.01, self.timer_callback)
        
    def steering_callback(self, msg):
        steering_offset = self.get_parameter('steering_offset').value
        steering_range = self.get_parameter('steering_range').value
        
        self.current_steering = int(msg.data * steering_range + steering_offset)
        self.send_command()

    def speed_callback(self, msg):
        max_speed = self.get_parameter('max_speed').value
        
        if msg.data > 0:
            self.current_speed = int(msg.data * max_speed)
            self.current_gear = 0  # 전진
        elif msg.data < 0:
            self.current_speed = int(-msg.data * max_speed)
            self.current_gear = 2  # 후진
        else:
            self.current_speed = 0
            self.current_gear = 1  # 중립
            
        self.send_command()

    def send_command(self):
        command = f'STX,0,0,{self.current_gear},{self.current_speed},{self.current_steering},0\r\n'
        self.serial.write(command.encode())

    def timer_callback(self):
        if self.serial.in_waiting:
            try:
                data = self.serial.readline().decode().strip()
                parts = data.split(',')
                if len(parts) == 7 and parts[0] == 'STX':
                    steering_msg = Float32()
                    steering_msg.data = float(parts[5])
                    self.steering_angle_pub.publish(steering_msg)
                    
                    speed_msg = Float32()
                    speed_msg.data = float(parts[4])
                    self.speed_pub.publish(speed_msg)
                    
            except Exception as e:
                self.get_logger().warning(f'Failed to read from Arduino: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ScaleVehicleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
