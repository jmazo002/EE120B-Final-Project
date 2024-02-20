import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import easyhid

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb
class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.subscriber = self.create_subscription(String, 'activation_signal', self.callback, 10)
        
        # Try to connect to the arm
        while True:
            try:
                # Stores an enumeration of all the connected USB HID devices
                en = easyhid.Enumeration()
                print(en)

                # return a list of devices based on the search parameters
                devices = en.find(vid=0x0483, pid=0x5750) 
                print(devices)

                assert len(devices) > 0
                self.dev = devices[0]

                # open a device
                self.dev.open()
                self.get_logger().info('Connected to Arm... Moving to default position')

                # Move to default position
                self.move_to(id=1, pos=500, time=1000)
                self.move_to(id=2, pos=500, time=1000)
                self.move_to(id=3, pos=-100, time=1000)
                self.move_to(id=4, pos=-100, time=1000)
                self.move_to(id=5, pos=500, time=1000)
                self.move_to(id=6, pos=200, time=1000)

                # If the connection and setup is successful, break out of the loop
                break
            except Exception:
                self.get_logger().info(f'Failed to connect to arm...')
                # If an exception occurs while trying to connect, wait for a short time and try again
                self.get_logger().info('Retrying in 10 seconds...')
                time.sleep(10)  # Import time module if not already imported
    def move_to(self, id, pos, time=0):
        """
        CMD_SERVO_MOVE
        0x55 0x55 len 0x03 [time_lsb time_msb, id, pos_lsb pos_msb]
        Servo position is in the range [0, 1000]
        """
        pos = pos+1000
        t_lsb, t_msb = itos(time)
        p_lsb, p_msb = itos(pos)
        self.dev.write([0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, id, p_lsb, p_msb])
    def callback(self, msg):
        if msg.data == "activate_recycle":
            self.get_logger().info('Activating arm for recycle...')
            # default position
            self.move_to(id=1, pos=500, time=1000)
            self.move_to(id=2, pos=500, time=1000)
            self.move_to(id=3, pos=-100, time=1000)
            self.move_to(id=4, pos=-100, time=1000)
            self.move_to(id=5, pos=500, time=1000)
            self.move_to(id=6, pos=200, time=1000)
            time.sleep(2)

            # grab motion
            self.move_to(id=5, pos=1000, time=1000)
            time.sleep(1)
            self.move_to(id=1, pos=500, time=1000)
            time.sleep(1)
            self.move_to(id=1, pos=1200, time=1000)
            time.sleep(2)

            # default position with grab
            self.move_to(id=2, pos=500, time=1000)
            self.move_to(id=3, pos=500, time=1000)
            self.move_to(id=4, pos=500, time=1000)
            self.move_to(id=5, pos=500, time=1000)
            self.move_to(id=6, pos=200, time=1000)
            
            # drop motion
            self.move_to(id=4, pos=1500, time=1000)
            self.move_to(id=5, pos=500, time=1000)
            self.move_to(id=6, pos=-100, time=1000)
            time.sleep(2)
            self.move_to(id=1, pos=500, time=1000)
            time.sleep(2)

            # default position
            self.move_to(id=1, pos=500, time=1000)
            self.move_to(id=2, pos=500, time=1000)
            self.move_to(id=3, pos=-100, time=1000)
            self.move_to(id=4, pos=-100, time=1000)
            self.move_to(id=5, pos=500, time=1000)
            self.move_to(id=6, pos=200, time=1000)
            time.sleep(1)

        elif msg.data == "activate_compost":
            self.get_logger().info('Deactivating arm for compost...')
            # default position
            self.move_to(id=1, pos=500, time=1000)
            self.move_to(id=2, pos=500, time=1000)
            self.move_to(id=3, pos=-100, time=1000)
            self.move_to(id=4, pos=-100, time=1000)
            self.move_to(id=5, pos=500, time=1000)
            self.move_to(id=6, pos=200, time=1000)
            time.sleep(2)

            # grab motion
            self.move_to(id=5, pos=1000, time=1000)
            time.sleep(1)
            self.move_to(id=1, pos=500, time=1000)
            time.sleep(1)
            self.move_to(id=1, pos=1000, time=1000)
            time.sleep(2)

            # default position with grab
            self.move_to(id=2, pos=500, time=1000)
            self.move_to(id=3, pos=500, time=1000)
            self.move_to(id=4, pos=500, time=1000)
            self.move_to(id=5, pos=500, time=1000)
            self.move_to(id=6, pos=200, time=1000)
            
            # drop motion
            self.move_to(id=4, pos=1500, time=1000)
            self.move_to(id=5, pos=500, time=1000)
            self.move_to(id=6, pos=500, time=1000)
            time.sleep(2)
            self.move_to(id=1, pos=500, time=1000)
            time.sleep(2)

            # default position
            self.move_to(id=1, pos=500, time=1000)
            self.move_to(id=2, pos=500, time=1000)
            self.move_to(id=3, pos=-100, time=1000)
            self.move_to(id=4, pos=-100, time=1000)
            self.move_to(id=5, pos=500, time=1000)
            self.move_to(id=6, pos=200, time=1000)
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()