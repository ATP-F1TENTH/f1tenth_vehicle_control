#
# node to manually control (and intervene) the vehicle
# Copyright 2024 philip.wette@hsbi.de
#

import rclpy

import rclpy.clock
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int16
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import aesthetic_control_interfaces.srv as ae_srv
import aesthetic_control_interfaces.msg as ae_msg
from flask import Flask, request
from enum import Enum
import time
import threading
import requests


import math

app = Flask(__name__)

TOPIC_OUT_DRIVE = "/drive"
TOPIC_BRAKE = "/commands/motor/brake"

TOPIC_IN_JOYSTICK = "/joy"
TOPIC_IN_DRIVE = "/to_drive"
TOPIC_IN_EMERGENCY_BRAKE = "/emergency_brake"
TOPIC_IN_ODOM = "/odom"
TOPIC_IN_RACE_POS = "/race_position"

LABTIMER_IP = "10.134.137.20"
LABTIMER_ADDRESS = f"http://{LABTIMER_IP}:8000/webhook"

class RaceState(Enum):
    STANDBY = 0  # default -> no race request behave normal  
    READY = 2 # got race request -> no autonomous driving
    RACING = 3 # GO GO GO
    CANCELED = 4
    FINISHED = 5

class VehicleControl(Node):
    def __init__(self: "VehicleControl"):
        super().__init__('vehicle_control')
        self.__in_safety_stop = False
        self.__in_autonomous_mode = False
        self.__speed_mode = "slow"
        self.__last_speed = 0.0
        self.__is_braking = False
        self.__brake_msg_cnt = 0
        self.__race_state = RaceState.STANDBY
        self.last_race_pos = -1

        #declare default parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('button_index_map.axis.angular_z',  0),
                ('button_index_map.axis.linear_x',   1),
                ('button_index_map.dead_man_switch', 4),
                ('button_index_map.fast_mode',       7),
                ('button_index_map.slow_mode',       5),
                ('button_index_map.start',           9),
                ('button_index_map.headlights',      0),
                ('button_index_map.headlights_off',  2),
                ('button_index_map.high_beam',       1),
                ('vehicle.fast_mode_top_speed',     15),
                ('vehicle.slow_mode_top_speed',      2),
                
            ])
        
        self.declare_parameter('car_id', '25')
        
        
        self.__car_id = int(self.get_parameter('car_id').value)
        
        #read parameters from yaml file
        self.__button_map = dict()
        self.__speed_limit = dict()
        self.__button_map["angular_z"]          = int(self.get_parameter('button_index_map.axis.angular_z').value)
        self.__button_map["linear_x"]           = int(self.get_parameter('button_index_map.axis.linear_x').value)
        self.__button_map["dead_man_switch"]    = int(self.get_parameter('button_index_map.dead_man_switch').value)
        self.__button_map["fast_mode"]          = int(self.get_parameter('button_index_map.fast_mode').value)
        self.__button_map["slow_mode"]          = int(self.get_parameter('button_index_map.slow_mode').value)
        self.__button_map["start"]              = int(self.get_parameter('button_index_map.start').value)
        self.__button_map["headlights"]         = int(self.get_parameter('button_index_map.headlights').value)
        self.__button_map["headlights_off"]     = int(self.get_parameter('button_index_map.headlights_off').value)
        self.__button_map["high_beam"]          = int(self.get_parameter('button_index_map.high_beam').value)
        self.__speed_limit["fast_mode"]         = int(self.get_parameter('vehicle.fast_mode_top_speed').value)
        self.__speed_limit["slow_mode"]         = int(self.get_parameter('vehicle.slow_mode_top_speed').value)

        #create publisher
        self.__publisher_vesc = self.create_publisher(AckermannDriveStamped, TOPIC_OUT_DRIVE, 10)
        self.__publisher_brake = self.create_publisher(Float64, TOPIC_BRAKE, 10)
        
        # define reentrant callback group to handle action and joy callback parallel
        cb_group_reentrant = ReentrantCallbackGroup()

        #create listeners
        self.__sub_laser    = self.create_subscription(AckermannDriveStamped, TOPIC_IN_DRIVE, self.callback_on_drive, 10, callback_group=cb_group_reentrant)
        self.__sub_joy      = self.create_subscription(Joy, TOPIC_IN_JOYSTICK, self.callback_on_joystick, 10,callback_group=cb_group_reentrant)
        self.__sub_brake    = self.create_subscription(Bool, TOPIC_IN_EMERGENCY_BRAKE, self.callback_on_emergency_brake, 10,callback_group=cb_group_reentrant)
        self.__sub_odom     = self.create_subscription(Odometry, TOPIC_IN_ODOM, self.callback_on_odom, 10, callback_group=cb_group_reentrant)
        self.__sub_race_pos = self.create_subscription(Int16, TOPIC_IN_RACE_POS, self.callback_on_race_pos, 10, callback_group=cb_group_reentrant)

        #try to connect to aesthetic control services
        """ Deactivate for simulation
        self.__services = {}
        self.__services["brakelights"] = self.create_client(ae_srv.BrakeLights, '/carAest/brake_lights')
        self.__services["headlights"] = self.create_client(ae_srv.Headlights, '/carAest/headlights')
        self.__services["highbeams"] = self.create_client(ae_srv.HighBeams, '/carAest/high_beam')
        self.__services["hazardlights"] = self.create_client(ae_srv.HazardLights, '/carAest/hazard_lights')
        self.__services["reverselights"] = self.create_client(ae_srv.ReverseLights, '/carAest/reverse_lights')
        self.__services["underglow"] = self.create_client(ae_srv.Underglow, '/carAest/underglow')
        
        for service in self.__services.values():
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{service} service not available, waiting again...')
        """
                
    
    ############################################################
    # Webhook Callbacks
    ############################################################
    
    # accept or reject client action goal request
    def _cb_get_ready(self):
        # check if everything is ready to paticipate in the race
        self.get_logger().info('Received Race request')
        
        
        # check if the vehicle is standing still, emergency brake is not active and the vehicle is not in autonomous mode  # float compare !
        if self.__in_autonomous_mode or self.__in_safety_stop or self.__last_speed > 0.0001:
            self.get_logger().info('Car not ready for race, rejecting request. in_autonomous_mode: {}, in_safety_stop: {}, last_speed: {}'.format(self.__in_autonomous_mode, self.__in_safety_stop, self.__last_speed) )
            self._send_ready(False)
            return 
        else:
            # change underglow color to orange to indicate readiness for race 
            # -> block autonomous mode
            # -> await dead_man_switch for some time
            self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([255,0,165])))
            
            
            self.__race_state = RaceState.READY
            
            timeout = 10
            now = self.get_clock().now()
            duration = rclpy.clock.Duration(seconds=timeout) 
            self.get_logger().info('Awaiting Dead_man_switch for {} seconds...'.format(timeout))
            
            while(now + duration > self.get_clock().now()):

                if self.__in_autonomous_mode:
                    # button was pressed -> ready for race
                    # signal to user with green underglow
                    # now await execute signal from client (lapTimer)
                    self.get_logger().info('Ready for race, awaiting execute signal...')
                    self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([0,255,0])))
                    self._send_ready(True)
                    return 
                pass
            
            # button was not pressed -> cancel goal
            self.get_logger().info('Dead_man_switch not pressed -> canceling race request')
            self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([255,0,0])))
            self.__race_state = RaceState.STANDBY
            
            self._send_ready(False)
            return 
        
        pass
    
    # accept or reject client action cancel request
    def _cb_race_cancel(self):
        self.__race_state = RaceState.CANCELED
        
    # execute client action goal -> do race
    def _cb_go_race(self):
        
        # change underglow color to blue to signal race start
        self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([0,0,255])))
        self.get_logger().info('Received execute request: GO GO GO')
        self.__race_state = RaceState.RACING
        
        # check if autonomous mode gets disabled -> only continue race when in autonomous mode
        # also stop when in emergency stop
        while self.__in_autonomous_mode and not self.__in_safety_stop:
            
            if(self.__race_state == RaceState.CANCELED):
                # stop race
                self.__race_state = RaceState.STANDBY
                
                boolMsg = Bool()
                boolMsg.data = True
                self.callback_on_emergency_brake(boolMsg)
                
                self.stop_vehicle()
                self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([0,0,0])))
                
                self.get_logger().info('ABORTING RACE: Got abort request')
                
                return 
            
            # periodically pubish position as feedback
            time.sleep(0.1)
            pass
        
        
        self.get_logger().info('ABORTING RACE: in_autonomous_mode: {}, in_safety_stop: {}'.format(self.__in_autonomous_mode, self.__in_autonomous_mode))
        self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([255,0,0])))
        
        # stopped vehicle -> signal abort to labTimer
        self._send_abort()
        self.__race_state = RaceState.STANDBY
        return 
        

        goal_handle.succeed()
        result = race_act.Race.Result()
        result.end_status = 1
        
        return result
    
    def _send_ready(self, is_ready = True):
        if type(is_ready) != bool:
            return
        
        self._post_request({'ready': is_ready})
    
    def _send_abort(self):
        self._post_request({'abort': True})
        
    def _send_race_pos(self, pos:int):
        if type(pos) != int:
            return
        if self.last_race_pos != pos :
            self._post_request({'pos': pos})
            self.last_race_pos = pos
    
    ############################################################
    # Node Callbacks
    ############################################################
    
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
            self.__in_autonomous_mode = False

        #start: disable safety stop
        if msg.buttons[self.__button_map["start"]] > 0:
            if self.__in_safety_stop:
                print("Disable Safety Stop", flush=True)
                self.__services["hazardlights"].call_async(ae_srv.HazardLights.Request(hazard_lights=False)) # turn off hazard lights
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
            angle = msg.axes[ self.__button_map["angular_z"] ] * math.radians(25.0) # amximum steering angle possible (tested)

            #vehicle speed
            speed = msg.axes[ self.__button_map["linear_x"] ] * self.__speed_limit["slow_mode"]
            if self.__speed_mode == "fast":
                speed = msg.axes[ self.__button_map["linear_x"] ] * self.__speed_limit["fast_mode"]

            #send message to vesc
            self.send_drive_message(speed=speed, angle=angle)
        
        # aesthetic control
        # headlights
        
        if msg.buttons[self.__button_map["headlights"]] > 0:
            self.__services["headlights"].call_async(ae_srv.Headlights.Request(headlights=True))
            
            self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([50,50,50])))
            
            
        if msg.buttons[self.__button_map["headlights_off"]] > 0:
            self.__services["headlights"].call_async(ae_srv.Headlights.Request(headlights=False))
            
        if msg.buttons[self.__button_map["high_beam"]] > 0:
            self.__services["highbeams"].call_async(ae_srv.HighBeams.Request(high_beams=True))


    def get_underglow_msg(self, color):
        #color = [0,0,0]
        glow_msg= ae_msg.UnderglowColor()
        glow_msg.set_underglow_color = color
        return glow_msg

    def callback_on_drive(self: "VehicleControl", msg: AckermannDriveStamped):
        if self.__in_autonomous_mode and not self.__in_safety_stop and not self.__race_state == RaceState.READY:

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
            #turn on blinking braking lights
            self.__services["brakelights"].call_async(ae_srv.BrakeLights.Request(brake_lights=True, flash=True))
            
    def callback_on_odom(self: "VehicleControl", msg: Odometry):
        curr_speed = msg.twist.twist.linear.x
        curr_speed = float(curr_speed)
        
        # turn on brake lights when decelerating with some threshold
        if (abs(self.__last_speed) > abs(curr_speed) + 0.01) and not self.__is_braking:
            self.__brake_msg_cnt +=1
            if self.__brake_msg_cnt == 3:
                self.__services["brakelights"].call_async(ae_srv.BrakeLights.Request(brake_lights=True))
                self.__is_braking = True
                self.__brake_msg_cnt = 0 
        elif (abs(self.__last_speed) < abs(curr_speed) - 0.01 or curr_speed < 0.00001) and self.__is_braking:
            self.__services["brakelights"].call_async(ae_srv.BrakeLights.Request(brake_lights=False))
            self.__is_braking = False
            
        
        # turn on reverse lights on when driving backwards
        if curr_speed < 0.0 and self.__last_speed >= 0.0:
            self.__services["reverselights"].call_async(ae_srv.ReverseLights.Request(reverse_lights=True))
        elif self.__last_speed < 0.0 and curr_speed >= 0.0:
            self.__services["reverselights"].call_async(ae_srv.ReverseLights.Request(reverse_lights=False))
        
        # handle hazardlights and blinking braking light when emergency breaking is called
        if self.__in_safety_stop and abs(curr_speed) < 0.1:
            self.__services["brakelights"].call_async(ae_srv.BrakeLights.Request(brake_lights=False))
            self.__services["hazardlights"].call_async(ae_srv.HazardLights.Request(hazard_lights=True))
            
        self.__last_speed = curr_speed
        
        
    def callback_on_race_pos(self, msg:Int16):
        self.race_pos = msg.data
        #self._send_race_pos(msg.data)
        
    ############################################################
    # Topic publishers
    ############################################################
    
    def stop_vehicle(self: "VehicleControl"):
        self.send_drive_message(0.0, 0.0)
        brake_current = Float64()
        brake_current.data = 20.0
        self.__publisher_brake.publish(brake_current)

    def send_drive_message(self: "VehicleControl", speed: float, angle : float):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "ego_racecar/laser"
        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(speed)
        self.__publisher_vesc.publish(drive_msg)
        
        
    def get_racetrack_position(self):
        if type(self.race_pos) != int:
            return
        self._post_request({'pos': self.race_pos}) 
    
    def _post_request(self, data:dict, url = LABTIMER_ADDRESS):
        #add car id to rquest
        data['car_id'] = self.__car_id
        
        try:
            response = requests.post(url, json=data)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error: {e}")
            return False
        return True
        
        
    def webhook_callback(self):
        print(request.json, flush=True)
        if request.method == 'POST':
            data = request.json
            
            if 'get_ready' in data:
                if data['get_ready'] == True:
                    self._cb_get_ready()
            if 'go' in data:
                if data['go'] == True:
                    self._cb_go_race()
            if 'abort' in data:
                if data['abort'] == True:
                    self._cb_race_cancel()
            if 'get_pos' in data:
                if data['get_pos'] == True:
                    self.get_racetrack_position()
            
        
        return "ok", 200
        
def start_webserver():
    app.run(host='0.0.0.0', port=8000)


def main(args=None):
    rclpy.init(args=args)

    vehicle_control = VehicleControl()
    app.add_url_rule('/webhook', methods=['POST'], view_func=vehicle_control.webhook_callback)

    thread_webserver = threading.Thread(target=start_webserver, daemon=True)
    thread_webserver.start() # todo kill webserver wen node is shut down?
    # use multithread executor to handle action callbacks and parallel 
    multithreaded_executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    rclpy.spin(vehicle_control, executor=multithreaded_executor)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()