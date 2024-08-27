#
# node to manually control (and intervene) the vehicle
# Copyright 2024 philip.wette@hsbi.de
#

import rclpy

from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import aesthetic_control_interfaces.srv as ae_srv
import aesthetic_control_interfaces.msg as ae_msg
from nav_msgs.msg import Odometry

import math

TOPIC_OUT_DRIVE = "/drive"
TOPIC_IN_ODOM = "/odom"
TOPIC_IN_JOYSTICK = "/joy"
TOPIC_IN_DRIVE = "/to_drive"
TOPIC_IN_EMERGENCY_BRAKE = "/emergency_brake"

class VehicleControl(Node):
    def __init__(self: "VehicleControl"):
        super().__init__('vehicle_control')
        self.__in_safety_stop = False
        self.__in_autonomous_mode = False
        self.__speed_mode = "slow"
        self.__last_speed = 0.0
        self.__is_currently_braking = False
        self.__deaccelerating_epochs = 0

        #declare default parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('button_index_map.axis.angular_z',  0),
                ('button_index_map.axis.linear_x',   3),
                ('button_index_map.dead_man_switch', 4),
                ('button_index_map.fast_mode',       7),
                ('button_index_map.slow_mode',       5),
                ('button_index_map.start',           9),
                ('vehicle.fast_mode_top_speed',      4),
                ('vehicle.slow_mode_top_speed',      1),
                
            ])
        
        #read parameters from yaml file
        self.__button_map = dict()
        self.__speed_limit = dict()
        self.__button_map["angular_z"]          = int(self.get_parameter('button_index_map.axis.angular_z').value)
        self.__button_map["linear_x"]           = int(self.get_parameter('button_index_map.axis.linear_x').value)
        self.__button_map["dead_man_switch"]    = int(self.get_parameter('button_index_map.dead_man_switch').value)
        self.__button_map["fast_mode"]          = int(self.get_parameter('button_index_map.fast_mode').value)
        self.__button_map["slow_mode"]          = int(self.get_parameter('button_index_map.slow_mode').value)
        self.__button_map["start"]              = int(self.get_parameter('button_index_map.start').value)
        self.__speed_limit["fast_mode"]         = int(self.get_parameter('vehicle.fast_mode_top_speed').value)
        self.__speed_limit["slow_mode"]         = int(self.get_parameter('vehicle.slow_mode_top_speed').value)

        #create publisher
        self.__publisher_vesc = self.create_publisher(AckermannDriveStamped, TOPIC_OUT_DRIVE, 10)

        #create listeners
        self.__sub_laser    = self.create_subscription(AckermannDriveStamped, TOPIC_IN_DRIVE, self.callback_on_drive, 10)
        self.__sub_joy      = self.create_subscription(Joy, TOPIC_IN_JOYSTICK, self.callback_on_joystick, 10)
        self.__sub_brake    = self.create_subscription(Bool, TOPIC_IN_EMERGENCY_BRAKE, self.callback_on_emergency_brake, 10)
        self.__sub_odom     = self.create_subscription(Odometry, TOPIC_IN_ODOM, self.callback_on_odom, 10)

        #create services
        self.__services = {}
        self.__services["brakelights"]      = self.create_client(ae_srv.BrakeLights,    '/carAest/brake_lights')
        self.__services["headlights"]       = self.create_client(ae_srv.Headlights,     '/carAest/headlights')
        self.__services["highbeams"]        = self.create_client(ae_srv.HighBeams,      '/carAest/high_beam')
        self.__services["hazardlights"]     = self.create_client(ae_srv.HazardLights,   '/carAest/hazard_lights')
        self.__services["reverselights"]    = self.create_client(ae_srv.ReverseLights,  '/carAest/reverse_lights')
        self.__services["underglow"]        = self.create_client(ae_srv.Underglow,      '/carAest/underglow')
        
        #connect to services
        for service in self.__services.values():
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{service} service not available, waiting again...')

        self.setup_vehicle()

    def setup_vehicle(self: "VehicleControl"):
        self.__services["headlights"].call_async(ae_srv.Headlights.Request(headlights=True))
        self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([86,190,215])))

    def callback_on_odom(self: "VehicleControl", msg: Odometry):
        #figure out if the car is currently decellerating based on odometry readings.

        current_speed = math.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)
        eps = 0.02                                                                                  #threshold for comparisons
        min_epochs = 3

        speed_diff = abs(current_speed) - abs(self.__last_speed)

        if speed_diff < -eps or (-eps < abs(current_speed) < eps):
            self.__deaccelerating_epochs = min(self.__deaccelerating_epochs+1, min_epochs)
            if self.__deaccelerating_epochs >= min_epochs:
                self.__is_currently_braking = True
        else:
            self.__deaccelerating_epochs = max(self.__deaccelerating_epochs-1, 0)
            if self.__deaccelerating_epochs == 0:
                self.__is_currently_braking = False


        self.__last_speed = current_speed
    
    def callback_on_joystick(self: "VehicleControl", msg: Joy):
        
        #dead man switch pressed: enable autonomous mode
        if msg.buttons[self.__button_map["dead_man_switch"]] > 0:
            if not self.__in_autonomous_mode:
                print("Entering Autonomous Mode", flush=True)
            self.__in_autonomous_mode = True
        else:
            if self.__in_autonomous_mode:
                self.stop_vehicle()
                print("Leaving Autonomous Mode", flush=True)
                self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([86,190,215])))
            self.__in_autonomous_mode = False

        #start: disable safety stop
        if msg.buttons[self.__button_map["start"]] > 0:
            if self.__in_safety_stop:
                print("Disable Safety Stop", flush=True)
            self.__in_safety_stop = False
        
        #speed modes
        if msg.buttons[self.__button_map["fast_mode"]] > 0:
            self.__speed_mode = "fast"
            print("Enable Fast Mode", flush=True)
        if msg.buttons[self.__button_map["slow_mode"]] > 0:
            self.__speed_mode = "slow"
            print("Enable Slow Mode", flush=True)

        #send manual controls to vesc
        if not self.__in_autonomous_mode and not self.__in_safety_stop:
            #steering angle
            angle = msg.axes[ self.__button_map["angular_z"] ] * math.radians(25) #25deg seems to be max possible steering angle

            #vehicle speed
            speed = msg.axes[ self.__button_map["linear_x"] ] * self.__speed_limit["slow_mode"]
            if self.__speed_mode == "fast":
                speed = msg.axes[ self.__button_map["linear_x"] ] * self.__speed_limit["fast_mode"]

            #send message to vesc
            self.send_drive_message(speed=speed, angle=angle)




    def callback_on_drive(self: "VehicleControl", msg: AckermannDriveStamped):
        if self.__in_autonomous_mode and not self.__in_safety_stop:

            #apply speed limit
            speed_limit = float(self.__speed_limit["slow_mode"])
            if self.__speed_mode == "fast":
                speed_limit = float(self.__speed_limit["fast_mode"])

            msg.drive.speed = min(msg.drive.speed, speed_limit)

            #publish ackermanndrive message to vesc
            self.__publisher_vesc.publish(msg)


    def callback_on_emergency_brake(self: "VehicleControl", msg: Bool):
        if msg.data is True:
            print("Locking up Vehicle. We are in safety stop.", flush=True)
            self.__in_safety_stop  = True
            self.stop_vehicle()

    def stop_vehicle(self: "VehicleControl"):
        self.send_drive_message(0.0, 0.0)

    def send_drive_message(self: "VehicleControl", speed: float, angle : float):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "ego_racecar/laser"
        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(speed)
        self.__publisher_vesc.publish(drive_msg)

        #handle brake lights depending on inputs to the vehicle
        if self.__last_speed > drive_msg.drive.speed:
            if not self.__is_currently_braking:
                self.__services["brakelights"].call_async(ae_srv.BrakeLights.Request(brake_lights=True))
            self.__is_currently_braking = True
        else:
            if self.__is_currently_braking:
                self.__services["brakelights"].call_async(ae_srv.BrakeLights.Request(brake_lights=False))
            self.__is_currently_braking = False

        self.__last_speed = drive_msg.drive.speed

    def get_underglow_msg(self, color):
        glow_msg= ae_msg.UnderglowColor()
        glow_msg.set_underglow_color = color
        return glow_msg


def main(args=None):
    rclpy.init(args=args)

    vehicle_control = VehicleControl()

    rclpy.spin(vehicle_control)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
