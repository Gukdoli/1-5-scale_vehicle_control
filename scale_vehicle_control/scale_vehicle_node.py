import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import serial

class ScaleVehicleNode(Node):
    def __init__(self):
        super().__init__('scale_vehicle_node')
        
        # Serial 연결
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Publishers (피드백값들)
        self.steering_angle_pub = self.create_publisher(Float32, 'steering_angle_feedback', 10)
        self.speed_pub = self.create_publisher(Float32, 'speed_feedback', 10)
        self.mode_pub = self.create_publisher(Int32, 'mode_status', 10)
        self.estop_pub = self.create_publisher(Int32, 'estop_status', 10)
        
        # Subscribers (제어 명령)
        self.steering_sub = self.create_subscription(Float32, 'steering_cmd', self.steering_callback, 10)
        self.speed_sub = self.create_subscription(Float32, 'speed_cmd', self.speed_callback, 10)
        
        # 아두이노 코드에서 가져온 파라미터들
        self.declare_parameters(
            namespace='',
            parameters=[
                ('steering_offset', 518),     # St_M
                ('steering_range', 380),      # St_D
                ('max_steering', 420),        # 최대 조향 제한
                ('max_speed', 250),           # 최대 속도
                ('max_speed_step', 15)        # Sp_MAX_R
            ]
        )
        
        # 현재 상태값들
        self.current_speed = 0
        self.current_steering = 518  # St_M (중립)
        self.current_gear = 0        # 0: 전진, 1: 중립, 2: 후진
        self.mode = 0               # 0: 수동, 1: 자율, 2: 디버그
        
        # 통신용 버퍼
        self.send_buffer = bytearray(13)
        self.send_buffer[0] = ord('S')
        self.send_buffer[1] = ord('T')
        self.send_buffer[2] = ord('X')
        self.send_buffer[11] = 0x0D
        self.send_buffer[12] = 0x0A
        
        # Timer for reading feedback
        self.create_timer(0.01, self.timer_callback)
        
    def steering_callback(self, msg):
        steering_offset = self.get_parameter('steering_offset').value
        steering_range = self.get_parameter('steering_range').value
        max_steering = self.get_parameter('max_steering').value
        
        # -1.0 ~ 1.0 값을 실제 조향값으로 변환
        raw_steering = int(msg.data * steering_range + steering_offset)
        self.current_steering = max(steering_offset - max_steering, 
                                  min(steering_offset + max_steering, raw_steering))
        self.send_command()

    def speed_callback(self, msg):
        max_speed = self.get_parameter('max_speed').value
        max_speed_step = self.get_parameter('max_speed_step').value
        
        # -1.0 ~ 1.0 값을 속도와 기어로 변환
        if msg.data > 0:
            self.current_speed = int(msg.data * max_speed_step)
            self.current_gear = 0  # 전진
        elif msg.data < 0:
            self.current_speed = int(-msg.data * max_speed_step)
            self.current_gear = 2  # 후진
        else:
            self.current_speed = 0
            self.current_gear = 1  # 중립
        
        self.send_command()

    def send_command(self):
        # STX AorM Les Lge Speed Steering Hal CR LF
        self.send_buffer[3] = self.mode & 0xFF
        self.send_buffer[4] = 0  # Les (Emergency Stop)
        self.send_buffer[5] = self.current_gear & 0xFF
        self.send_buffer[6] = (self.current_speed >> 8) & 0xFF
        self.send_buffer[7] = self.current_speed & 0xFF
        self.send_buffer[8] = (self.current_steering >> 8) & 0xFF
        self.send_buffer[9] = self.current_steering & 0xFF
        self.send_buffer[10] = 0  # Hal (Alive)
        
        self.serial.write(self.send_buffer)

    def timer_callback(self):
        if self.serial.in_waiting:
            try:
                data = self.serial.read(13)  # STX AorM Les Lge RPM Steer_Feed Lal CR LF
                if len(data) == 13 and data[0] == ord('S') and data[1] == ord('T') and data[2] == ord('X'):
                    # 피드백 메시지 발행
                    mode_msg = Int32()
                    mode_msg.data = data[3]
                    self.mode_pub.publish(mode_msg)
                    
                    estop_msg = Int32()
                    estop_msg.data = data[4]
                    self.estop_pub.publish(estop_msg)
                    
                    rpm = (data[6] << 8) | data[7]
                    speed_msg = Float32()
                    speed_msg.data = float(rpm)
                    self.speed_pub.publish(speed_msg)
                    
                    steer = (data[8] << 8) | data[9]
                    steering_msg = Float32()
                    steering_msg.data = float(steer)
                    self.steering_angle_pub.publish(steering_msg)
                    
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
