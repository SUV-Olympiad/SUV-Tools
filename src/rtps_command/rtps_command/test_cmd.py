import rclpy
from rclpy.node import Node


from px4_msgs.msg import VehicleCommand
from pynput.keyboard import Key, Listener

 

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('px4_command_publisher')

        self._NUM = 20

        self._pub = []
        for i in range(self._NUM):
            name = f'/vehicle{i+1}/in/VehicleCommand'
            print(name)
            pub = self.create_publisher(VehicleCommand, name, 10)
            self._pub.append(pub)

def on_press(key):
    global cmd_publisher
    try:
        if(key.char == 'q'):
            print("q pressed! send ARM command")
            for i in range(cmd_publisher._NUM):
                arm_cmd = VehicleCommand()
                arm_cmd.target_system = i + 1
                arm_cmd.command = 400
                arm_cmd.param1 = 1.0
                arm_cmd.confirmation = True
                arm_cmd.from_external = True
                cmd_publisher._pub[i].publish(arm_cmd)


        elif(key.char == 'w'):
            print("w pressed! send DISARM command")
            disarm_cmd = VehicleCommand()
            disarm_cmd.target_system = 1
            disarm_cmd.command = 400
            disarm_cmd.param1 = 0.0
            disarm_cmd.confirmation = True
            disarm_cmd.from_external = True
            cmd_publisher.publisher_.publish(disarm_cmd)

        elif(key.char == 'a'):
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
                #takeoff_cmd.param5 = 47.3977508 + 0.0001 * i
                #takeoff_cmd.param6 = 8.5456069
                takeoff_cmd.param5 = float('inf')
                takeoff_cmd.param6 = float('inf')
                takeoff_cmd.param7 = 495.0
                takeoff_cmd.confirmation = True
                takeoff_cmd.from_external = True
                cmd_publisher._pub[i].publish(takeoff_cmd)
                #cmd_publisher.publisher_.publish(takeoff_cmd)
                #cmd_publisher.publisher2_.publish(takeoff_cmd)

        elif(key.char == 's'):
            print("s pressed! send Landing command")
            landing_cmd = VehicleCommand()
            landing_cmd.target_system = 1
            landing_cmd.command = 21
            landing_cmd.from_external = True
            cmd_publisher.publisher_.publish(landing_cmd)

    except Exception as e:

        print(e)

 

def main(args=None):

    global cmd_publisher

    rclpy.init(args=args)
    cmd_publisher = CmdPublisher()

    with Listener(on_press=on_press) as listener:
        listener.join()

    #rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("ARM : q\nDISARM : w\n")
    main()
