import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32, Int32
import serial

class ScaleVehicleNode(Node):
    def __init__(self):
        super().__init__('scale_vehicle_node')
        
        # Subscriber
        self.ackermann_sub = self.create_subscription(
            AckermannDrive,
            'ackermann_cmd',
            self.ackermann_callback,
            10
        )
        
        # Publishers for feedback
        self.steering_fb_pub = self.create_publisher(Float32, '/vehicle/steering_angle', 10)
        self.speed_fb_pub = self.create_publisher(Float32, '/vehicle/speed', 10)
        self.gear_fb_pub = self.create_publisher(Int32, '/vehicle/gear', 10)
        self.mode_fb_pub = self.create_publisher(Int32, '/vehicle/mode', 10)
        
        # 현재 상태값
        self.current_speed = 0
        self.current_steering = 0
        self.current_gear = 0
        self.mode = 1
        self.alive_counter = 0
        
        # 시리얼 연결
        try:
            self.serial = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=0.1
            )
            self.get_logger().info('Serial port opened successfully')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {str(e)}')
            raise e
        
        # 명령 전송 및 피드백 읽기 타이머 (50Hz)
        self.create_timer(0.02, self.timer_callback)

    def ackermann_callback(self, msg):
        """Ackermann 명령 처리"""
        # 속도 처리
        speed = abs(msg.speed)
        if speed > 0:
            self.current_speed = int(min(speed * 15, 15))
            self.current_gear = 0 if msg.speed > 0 else 2
        else:
            self.current_speed = 0
            self.current_gear = 1

        # 조향각 처리
        steering = int((msg.steering_angle / 0.5) * 2000)
        self.current_steering = max(-2000, min(2000, steering))

    def send_command(self):
        """LLC 프로토콜 명령 전송"""
        try:
            message = bytearray(b'STX')
            message.extend([
                self.mode,                           # Mode
                0,                                  # Emergency Stop
                self.current_gear,                  # Gear
                (self.current_speed >> 8) & 0xFF,   # Speed upper
                self.current_speed & 0xFF,          # Speed lower
                (self.current_steering >> 8) & 0xFF, # Steering upper
                self.current_steering & 0xFF,        # Steering lower
                self.alive_counter,                 # Alive counter
                0x0D,                              # CR
                0x0A                               # LF
            ])
            
            self.serial.write(message)
            self.alive_counter = (self.alive_counter + 1) % 256
            
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {str(e)}')

    def timer_callback(self):
        """명령 전송 및 피드백 처리"""
        self.send_command()
        
        # 피드백 데이터 읽기
        if self.serial.in_waiting >= 13:
            try:
                data = self.serial.read(13)
                
                if (len(data) == 13 and 
                    data[0:3] == b'STX' and 
                    data[11] == 0x0D and 
                    data[12] == 0x0A):
                    
                    # 모드 피드백
                    mode_msg = Int32()
                    mode_msg.data = data[3]
                    self.mode_fb_pub.publish(mode_msg)
                    
                    # 기어 피드백
                    gear_msg = Int32()
                    gear_msg.data = data[5]
                    self.gear_fb_pub.publish(gear_msg)
                    
                    # RPM 피드백
                    rpm = (data[6] << 8) | data[7]
                    speed_msg = Float32()
                    speed_msg.data = float(rpm)
                    self.speed_fb_pub.publish(speed_msg)
                    
                    # 조향각 피드백 (-2000~2000 범위에서 라디안으로 변환)
                    steer = (data[8] << 8) | data[9]
                    steering_msg = Float32()
                    steering_msg.data = float(steer) * 0.5 / 2000.0  # 라디안으로 변환
                    self.steering_fb_pub.publish(steering_msg)
                    
                    
            except Exception as e:
                self.get_logger().error(f'Failed to read feedback: {str(e)}')
                self.serial.reset_input_buffer()

    def __del__(self):
        """종료 처리"""
        if hasattr(self, 'serial') and self.serial.is_open:
            final_message = bytearray(b'STX')
            final_message.extend([1, 0, 1, 0, 0, 0, 0, 0, 0x0D, 0x0A])
            self.serial.write(final_message)
            self.serial.close()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ScaleVehicleNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
