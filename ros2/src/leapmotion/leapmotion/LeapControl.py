import LeapFuncV3 as LF
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from suv_msgs.msg import Leap

class LeapPublisher(Node):

    def __init__(self):
        super().__init__('leapmotion')
        qos_profile = QoSProfile(depth=10)
        self.leap_publisher = self.create_publisher(Leap, 'leapmotion', qos_profile)
        self.timer = self.create_timer(1, self.publish_helloworld_msg)
        self.count = 0
        self.LeapMotion = LeapMotion()

    def publish_helloworld_msg(self):
        msg = Leap()
        msg.roll = float(self.LeapMotion.listener.rh_value[0])
        msg.pitch = float(self.LeapMotion.listener.rh_value[1])
        msg.yaw = float(self.LeapMotion.listener.rh_value[2])
        msg.height = float(self.LeapMotion.listener.rh_value[3])
        msg.grip = float(self.LeapMotion.listener.rh_value[4])
        self.leap_publisher.publish(msg)
        self.get_logger().info('Published message: ' \
                                '[roll: {0}, pitch: {1}, yaw: {2}, height: {3}, grip: {4}' \
                                .format(msg.roll, msg.pitch, msg.yaw, msg.height, msg.grip))
        self.count += 1



class LeapMotion:
    listener = LF.SampleListener()
    controller = LF.Leap.Controller()
    controller.add_listener(listener)
    is_takeoff = False
    is_ok = True

    def callback(self):
        pass


    def before_move(self):
        self.list_clean()
        self.is_ok = False

    def control_drone(self):
        val = self.listener.rh_value

        lr_val = 0
        ud_val = 0
        fb_val = 0
        turn_val = 0
        to_land_check = 0
        lr_name = 'N'
        fb_name = 'N'
        ud_name = 'N'

        # refine roll value

        if val[0] < 15 and val[0] > -15:
            lr_val = 0
        elif val[0] > 65:
            lr_name = 'Right '
            lr_val = -100
        elif val[0] < -65:
            lr_name = 'Left '
            lr_val = 100
        elif val[0] > 15:  # right
            lr_name = 'Right '
            lr_val = int(((val[0] - 15) / 50 * 90 + 10) * (-1))
        elif val[0] < -15:  # left
            lr_name = 'Left '
            lr_val = int(((val[0] + 15) / 50 * 90 + 10) * (-1))

        # refine pitch value
        if val[1] < 20 and val[1] > -15:
            fb_val = 0
        elif val[1] > 50:
            fb_name = 'Back '
            fb_val = 100
        elif val[1] < -45:
            fb_name = 'Forward '
            fb_val = -100
        elif val[1] > 20:  # back
            fb_name = 'Back '
            fb_val = int((val[1] - 20) / 30 * 90 + 10)
        elif val[1] < -15:  # forward
            fb_name = 'Forward '
            fb_val = int((val[1] + 15) / 30 * 90 - 10)

            # ud_val = val[2]
        # 80 / 120

        if val[2] < 200 and val[2] > 100:
            ud_val = 0
        elif val[2] > 300:
            ud_name = 'Up '
            ud_val = 100
        elif val[2] < 20:
            ud_name = 'Down '
            ud_val = -100
        elif val[2] > 200:  # up
            ud_name = 'Up '
            ud_val = int(((val[2] - 200) / 100 * 90 + 10))
        elif val[2] < 100:  # down
            ud_name = 'Down '
            ud_val = int(((-1) * val[2] + 100) / 80 * 90 + 10) * (-1)
        else:
            print("WTF")
        # turn_val = val[3]

        if val[3] > 35:
            turn_val = 50
        elif val[3] < -35:
            turn_val = -50

        to_land_check = val[4]
        """
        we must normalize lr ud fb value
        plz normalize lr ud fb value before use this function.
        roll : max : -60 / 60, min : -10 / 10
        pitch : max : -60 / 40, min :- 10 / 10
        height : ?
        turn : ?
        """
        if not self.is_takeoff:
            if to_land_check > 0.8:
                if self.is_ok:
                    print("takeoff")
                    self.is_takeoff = True
        else:
            if to_land_check > 0.8:
                if self.is_ok:
                    print("land")
                    self.is_takeoff = False
            else:
                if self.is_ok:
                    lr = lr_name + str(lr_val)
                    fb = fb_name + str(fb_val)
                    ud = ud_name + str(ud_val)
                    send_value = (lr, fb, ud, turn_val)
                    print(send_value)

    def list_clean(self):
        pass


def leap_test():
    print("test")
    test = LeapMotion()
    while 1:
        test.control_drone()
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = LeapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

# def main():
#     pass

if __name__ == "__main__":
    # leap_test()
    main()
