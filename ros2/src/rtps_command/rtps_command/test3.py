import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
import time


from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus
from pynput.keyboard import Key, Listener


class CmdPublisher(Node):
    def __init__(self):
        super().__init__('px4_command_publisher')

        self._NUM = 20

        self._pub = []
        self.position_sub = [None] * self._NUM

        self._location = [[None, None, None]] * self._NUM
        self._arming_status = [None] * self._NUM
        self._takeoff_time = [None] * self._NUM
        self.moving_value = 0

        for i in range(self._NUM):
            name = f'/vehicle{i + 1}/in/VehicleCommand'
            print(name)
            pub = self.create_publisher(VehicleCommand, name, 10)
            self._pub.append(pub)

    def get_position(self):
        for i in range(self._NUM):
            #print(self._location)
            msg = None
            sub = self.create_subscription(VehicleLocalPosition, f'/vehicle{i + 1}/out/VehicleLocalPosition', lambda msg: self.position_listener(msg, i), 10)
            rclpy.spin_once(self)
            self.destroy_subscription(sub)

    def get_status(self):
        for i in range(self._NUM):
            # print(self._arming_status)
            #print(self._takeoff_time)
            sub = self.create_subscription(VehicleStatus, f'/vehicle{i + 1}/out/VehicleStatus', lambda msg: self.status_listener(msg, i), 10)
            rclpy.spin_once(self)
            self.destroy_subscription(sub)

    def position_listener(self, msg, i):
        self._location[i] = [msg.ref_lat, msg.ref_lon, msg.ref_alt]

    def status_listener(self, msg, i):
        self._arming_status[i] = msg.arming_state
        self._takeoff_time[i] = msg.takeoff_time

def on_press(key):
    global cmd_publisher

    cmd_publisher.moving_value += 0.00001

    try:
        if (key.char == 'q'):
            cmd_publisher.get_logger().info("q pressed! send ARM command")
            for i in range(cmd_publisher._NUM):
                arm_cmd = VehicleCommand()
                arm_cmd.target_system = i + 1
                arm_cmd.command = 400
                arm_cmd.param1 = 1.0
                arm_cmd.confirmation = True
                arm_cmd.from_external = True
                cmd_publisher._pub[i].publish(arm_cmd)
            cmd_publisher.get_status()
            time.sleep(3)
            cmd_publisher.get_logger().info("checking vehicle arming status")
            for i in range(cmd_publisher._NUM):
                if cmd_publisher._arming_status[i] != 2:
                    cmd_publisher.get_logger().warn(f'Vehicle{i+1}\'s arming status is 1. send ARM command')
                    arm_cmd = VehicleCommand()
                    arm_cmd.target_system = i + 1
                    arm_cmd.command = 400
                    arm_cmd.param1 = 1.0
                    arm_cmd.confirmation = True
                    arm_cmd.from_external = True
                    cmd_publisher._pub[i].publish(arm_cmd)
                    cmd_publisher.get_status()
                    if cmd_publisher._arming_status[i] == 2:
                        cmd_publisher.get_logger().info(f'Vehicle{i+1} is armed')
                else:
                    cmd_publisher.get_logger().info(f'Vehicle{i+1} is armed')

        elif (key.char == 'w'):
            print("w pressed! send DISARM command")
            disarm_cmd = VehicleCommand()
            disarm_cmd.target_system = 1
            disarm_cmd.command = 400
            disarm_cmd.param1 = 0.0
            disarm_cmd.confirmation = True
            disarm_cmd.from_external = True
            cmd_publisher.publisher_.publish(disarm_cmd)

        elif (key.char == 'a'):
            print("a pressed! send Take-off command")
            for i in range(cmd_publisher._NUM):
                takeoff_cmd = VehicleCommand()
                takeoff_cmd.target_system = 1 + i
                takeoff_cmd.command = 22
                takeoff_cmd.param1 = -1.0
                takeoff_cmd.param2 = 0.0
                takeoff_cmd.param3 = 0.0
                takeoff_cmd.param4 = 0.0
                # lat: 47.39775103965341
                # lon: 8.545607598150605
                # alt: 488.1470947265625
                # takeoff_cmd.param5 = 47.3977508 + 0.0001 * i
                # takeoff_cmd.param6 = 8.5456069
                takeoff_cmd.param5 = float('inf')
                takeoff_cmd.param6 = float('inf')
                takeoff_cmd.param7 = 495.0
                takeoff_cmd.confirmation = True
                takeoff_cmd.from_external = True
                cmd_publisher._pub[i].publish(takeoff_cmd)
                # cmd_publisher.publisher_.publish(takeoff_cmd)
                # cmd_publisher.publisher2_.publish(takeoff_cmd)
            time.sleep(2)
            cmd_publisher.get_status()
            cmd_publisher.get_logger().info("checking vehicle takeoff status")
            for i in range(cmd_publisher._NUM):
                if cmd_publisher._takeoff_time[i] == 0:
                    cmd_publisher.get_logger().warn(f'Vehicle{i+1}\'s takeoff time is 0. send takeoff command')
                    takeoff_cmd = VehicleCommand()
                    takeoff_cmd.target_system = 1 + i
                    takeoff_cmd.command = 22
                    takeoff_cmd.param1 = -1.0
                    takeoff_cmd.param2 = 0.0
                    takeoff_cmd.param3 = 0.0
                    takeoff_cmd.param4 = 0.0
                    takeoff_cmd.param5 = float('inf')
                    takeoff_cmd.param6 = float('inf')
                    takeoff_cmd.param7 = 495.0
                    takeoff_cmd.confirmation = True
                    takeoff_cmd.from_external = True
                    cmd_publisher._pub[i].publish(takeoff_cmd)
                    cmd_publisher.get_status()

        elif (key.char == 's'):
            print("s pressed! send Landing command")
            landing_cmd = VehicleCommand()
            landing_cmd.target_system = 1
            landing_cmd.command = 21
            landing_cmd.from_external = True
            cmd_publisher.publisher_.publish(landing_cmd)

        elif (key.char == 'm'):
            print("m pressed! send moving command")
            cmd_publisher.get_position()
            for i in range(cmd_publisher._NUM):
                move_cmd = VehicleCommand()
                move_cmd.target_system = i + 1
                move_cmd.command = 192
                move_cmd.param1 = -1.0
                move_cmd.param2 = 1.0
                move_cmd.param3 = 0.0
                move_cmd.param4 = float('nan')
                move_cmd.param5 = cmd_publisher._location[i][0]
                move_cmd.param6 = cmd_publisher._location[i][1] + cmd_publisher.moving_value
                move_cmd.param7 = 495.0
                move_cmd.confirmation = True
                move_cmd.from_external = True
                cmd_publisher._pub[i].publish(move_cmd)


    except Exception as e:

        print(e)



def main(args=None):
    global cmd_publisher

    rclpy.init(args=args)
    cmd_publisher = CmdPublisher()

    with Listener(on_press=on_press) as listener:
        listener.join()

    #rclpy.spin(cmd_publisher)
    cmd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print("ARM : q\nDISARM : w\n")
    main()