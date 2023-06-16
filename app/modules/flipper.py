import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from ur_dashboard_msgs.srv import Load
from std_srvs.srv import Trigger
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Pose
from sensor_msgs.msg import PointCloud, PointCloud2, PointField, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from ur_msgs.msg import IOStates, Digital
import tf2_ros 
from tf2_ros import TransformException
import time
from numpy import transpose
import numpy as np
import asyncio
import quaternion
import transforms3d
import threading
from moveit2_ur5.msg import PoseAndVelo
from enum import Enum

class WorkInProgress(Exception):
    pass

class FlipperStates(Enum):
    IDLE = 'idle'
    SCAN = 'scan'
    FLIP = 'flip'
    DELIVER = 'deliver'

class Flipper(object):
    def __init__(self, node):
        self.state = FlipperStates.IDLE

        self.node = node
        self.prev_digital_in_state = False
        self.node.get_logger().info('starting flip demo!')
        # subscribers
        self.io_sub = self.node.create_subscription(IOStates, 'io_and_status_controller/io_states', self.io_callback,10)
        # publishers
        self.pointcloud2_pub = self.node.create_publisher(PointCloud2, 'pointcloud2', 10)
        # self.plotdata_pub = self.node.create_publisher(Plotdata, 'Plotdata', 10)
        self.move_request = self.node.create_publisher(PoseAndVelo, 'move_request', 10)
        # clients
        # self.cli = self.node.create_client(WenglorSRV, 'Wenglor2dScanSRV')
        # ros parameters
        self.node.declare_parameter('fancy_move',1)
        self.node.declare_parameter('sweep_scan',1)
        # set fancy move on or off according to parameter value
        # fancy move is a circular motion in the yz plane
        self.fancy_move = self.node.get_parameter('fancy_move').get_parameter_value().integer_value
        self.sweep_scan_enable = self.node.get_parameter('sweep_scan').get_parameter_value().integer_value
        if self.fancy_move:
            self.node.get_logger().info('fancy_move is enabled')
        else:
            self.node.get_logger().info('fancy_move is disabled')
        
        #init transfrom listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, node)
        try: 
            transform = self.tfBuffer.lookup_transform('wenglor_mount', 'world', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.node.get_logger().info('Sensor transform initialization')
        # move the ros node to seperate thread,
        # this way the ros node can be spinned continuously
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
            
        #init pointfields data structure for pointcloud2 msg
        self.fields = [PointField(),PointField(),PointField(),PointField()]
        self.fields[0].name = 'x'
        self.fields[0].offset = 0
        self.fields[0].datatype = PointField.FLOAT32
        self.fields[0].count = 1
        self.fields[1].name = 'y'
        self.fields[1].offset = 4
        self.fields[1].datatype = PointField.FLOAT32
        self.fields[1].count = 1
        self.fields[2].name = 'z'
        self.fields[2].offset = 8
        self.fields[2].datatype = PointField.FLOAT32
        self.fields[2].count = 1
        self.fields[3].name = 'Intensity'
        self.fields[3].offset = 12
        self.fields[3].datatype = PointField.FLOAT32
        self.fields[3].count = 1

        # data buffers for the pointcloud
        self.measure_points = np.zeros((2048,4))
        self.current_points = np.zeros((2048,4))
        self.current_points_base = np.zeros((1,4))

    def __del__(self):
        # to stop node thread
        self.executor_thread.join()

    # loads the external control program on ur5 via dashboard client
    # if success: starts the external control program via start_stop_ext_control()
    def connect_to_robot(self):
        # wait a few seconds for the other programs to start
        time.sleep(3)
        # first check if wenglor service is available
        wait_max_seconds = 4
        if self.wait_for_wenglor_service(wait_max_seconds):
            self.node.get_logger().info('Attempting to load external control program')
            request = Load.Request()
            result = Load.Response()
            request.filename = "external_control.urp"
            cli_load = self.node.create_client(Load, 'dashboard_client/load_program')
            counter = 0
            while not cli_load.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('dashboard service not available, waiting again...')
                counter += 1
                if counter > 3:
                    self.node.get_logger().info('dashboard service remains not available...')
                    break
            future = cli_load.call_async(request)
            counter = 0
            while future.done() != True: 
                time.sleep(0.1)
                counter += 1
                if counter > 15:
                    self.node.get_logger().info('Received no response from ur5, assuming program could not be loaded..')
                    break
            try:
                response = future.result()
                if response.success:
                    self.node.get_logger().info('Successfuly loaded external control on UR5')
                    if self.start_stop_ext_control(True):
                        return True
                else:
                    self.node.get_logger().info(
                            'Load error %r' % (response.answer))
                    return False
            except (asyncio.InvalidStateError, asyncio.CancelledError, AttributeError):
                return False
        else:
            return False

    # starts or stops program which has been loaded
    def start_stop_ext_control(self, start):
        request = Trigger.Request()
        result = Trigger.Response()
        if start:
            self.node.get_logger().info('Attempting to start external control program')
            cli_play = self.node.create_client(Trigger, 'dashboard_client/play')
            Future = cli_play.call_async(request) 
        else:
            self.node.get_logger().info('Attempting to stop external control program')
            cli_stop = self.node.create_client(Trigger, 'dashboard_client/stop')
            Future = cli_stop.call_async(request)
        counter = 0
        while Future.done() != True: 
                time.sleep(0.1)
                counter += 1
                if counter > 10:
                    self.node.get_logger().info('Received no response from ur5, assuming program could not be stopped or started..')
                    break
        try:
            response = Future.result()
            if response.success:
                self.node.get_logger().info(
                        'message: %r' % (response.message))
                return True
            else:
                self.node.get_logger().info(
                    'error: %r' % (response.message))
                return False
        except:
            return False

    def io_callback(self,msg):
        pin = msg.digital_in_states[0].pin
        if not self.prev_digital_in_state:
            if msg.digital_in_states[0].state:
                if self.state != FlipperStates.IDLE:
                    self.node.get_logger().info(f'Demonstrator set to {self.state.value} mode..')
                    raise WorkInProgress()
                    self.start_stop_ext_control(False)
                self.node.get_logger().info('Demonstrator is going to resume..')
                self.state = FlipperStates.FLIP
                self.start_stop_ext_control(True)
        self.prev_digital_in_state = msg.digital_in_states[0].state
        #self.node.get_logger().info('digital pin %i state: %i' % (pin, self.pauze_state))
        
    def flip(self, piston_sensors):
        if self.state != FlipperStates.IDLE:
            raise WorkInProgress()
        self.state = FlipperStates.FLIP

        if np.array_equal(piston_sensors, np.array([1,0,1,0])) and self.lowering_piston:
            self.lowering_piston = False
            self.state = FlipperStates.SCAN
            self.scan()
        elif np.array_equal(piston_sensors, np.array([1,0,1,0])):
            self.raise_pistons()
        elif np.array_equal(piston_sensors, np.array([0,1,0,1])):
            self.lowering_piston = True
            self.lower_pistons()
        else:
            raise Exception("Unexpected piston sensor state")

    def scan(self):
        if self.state != FlipperStates.IDLE:
            raise WorkInProgress()
        self.state = FlipperStates.SCAN

        # todo:: send ros message to UR5 to scan + callback function confirmation from FTP server

    def deliver(self):
        if self.state != FlipperStates.IDLE:
            raise WorkInProgress()
        self.state = FlipperStates.DELIVER

        # todo:: send ros message with position + callback function confirmation from pick up robot (ROS reply)




# # init
#     def __init__(self,node):
#         self.node = node
#         self.pauze_state = True
#         self.prev_digital_in_state = False
#         self.node.get_logger().info('starting Weld demonstrator!')
#         # subscribers
#         self.io_sub = self.node.create_subscription(IOStates, 'io_and_status_controller/io_states', self.io_callback,10)
#         # publishers
#         self.pointcloud2_pub = self.node.create_publisher(PointCloud2, 'pointcloud2', 10)
#         self.plotdata_pub = self.node.create_publisher(Plotdata, 'Plotdata', 10)
#         self.move_request = self.node.create_publisher(PoseAndVelo, 'move_request', 10)
#         # clients
#         self.cli = self.node.create_client(WenglorSRV, 'Wenglor2dScanSRV')
#         # ros parameters
#         self.node.declare_parameter('fancy_move',1)
#         self.node.declare_parameter('sweep_scan',1)
#         # set fancy move on or off according to parameter value
#         # fancy move is a circular motion in the yz plane
#         self.fancy_move = self.node.get_parameter('fancy_move').get_parameter_value().integer_value
#         self.sweep_scan_enable = self.node.get_parameter('sweep_scan').get_parameter_value().integer_value
#         if self.fancy_move:
#             self.node.get_logger().info('fancy_move is enabled')
#         else:
#             self.node.get_logger().info('fancy_move is disabled')
        
#         #init transfrom listener
#         self.tfBuffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.tfBuffer, node)
#         try: 
#             transform = self.tfBuffer.lookup_transform('wenglor_mount', 'world', rclpy.time.Time())
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             self.node.get_logger().info('Sensor transform initialization')
#         # move the ros node to seperate thread,
#         # this way the ros node can be spinned continuously
#         self.executor = rclpy.executors.MultiThreadedExecutor()
#         self.executor.add_node(node)
#         self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
#         self.executor_thread.start()
            
#         #init pointfields data structure for pointcloud2 msg
#         self.fields = [PointField(),PointField(),PointField(),PointField()]
#         self.fields[0].name = 'x'
#         self.fields[0].offset = 0
#         self.fields[0].datatype = PointField.FLOAT32
#         self.fields[0].count = 1
#         self.fields[1].name = 'y'
#         self.fields[1].offset = 4
#         self.fields[1].datatype = PointField.FLOAT32
#         self.fields[1].count = 1
#         self.fields[2].name = 'z'
#         self.fields[2].offset = 8
#         self.fields[2].datatype = PointField.FLOAT32
#         self.fields[2].count = 1
#         self.fields[3].name = 'Intensity'
#         self.fields[3].offset = 12
#         self.fields[3].datatype = PointField.FLOAT32
#         self.fields[3].count = 1

#         # data buffers for the pointcloud
#         self.measure_points = np.zeros((2048,4))
#         self.current_points = np.zeros((2048,4))
#         self.current_points_base = np.zeros((1,4))
        

    

#     def move_to_init_pos(self):
#         self.send_move_request(0.2,-0.225, 0.1, 0.45,3.14, -1.571, 3.14)
#         self.node.get_logger().info('move request to init pose')
#         time.sleep(5)

#     # check if the wenglor service is available, wait for max_counter_val seconds
#     def wait_for_wenglor_service(self, max_counter_val):
#         counter = 0
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.node.get_logger().info('wenglor service not available, waiting again...')
#             counter += 1
#             if counter > max_counter_val:
#                 self.node.get_logger().info('wenglor service remains unavailable...')
#                 return False
#         return True
    
#     # loads the external control program on ur5 via dashboard client
#     # if success: starts the external control program via start_stop_ext_control()
#     def connect_to_robot(self):
#         # wait a few seconds for the other programs to start
#         time.sleep(3)
#         # first check if wenglor service is available
#         wait_max_seconds = 4
#         if self.wait_for_wenglor_service(wait_max_seconds):
#             self.node.get_logger().info('Attempting to load external control program')
#             request = Load.Request()
#             result = Load.Response()
#             request.filename = "external_control.urp"
#             cli_load = self.node.create_client(Load, 'dashboard_client/load_program')
#             counter = 0
#             while not cli_load.wait_for_service(timeout_sec=1.0):
#                 self.node.get_logger().info('dashboard service not available, waiting again...')
#                 counter += 1
#                 if counter > 3:
#                     self.node.get_logger().info('dashboard service remains not available...')
#                     break
#             future = cli_load.call_async(request)
#             counter = 0
#             while future.done() != True: 
#                 time.sleep(0.1)
#                 counter += 1
#                 if counter > 15:
#                     self.node.get_logger().info('Received no response from ur5, assuming program could not be loaded..')
#                     break
#             try:
#                 response = future.result()
#                 if response.success:
#                     self.node.get_logger().info('Successfuly loaded external control on UR5')
#                     if self.start_stop_ext_control(True):
#                         return True
#                 else:
#                     self.node.get_logger().info(
#                             'Load error %r' % (response.answer))
#                     return False
#             except (asyncio.InvalidStateError, asyncio.CancelledError, AttributeError):
#                 return False
#         else:
#             return False

#     # starts or stops program which has been loaded
#     def start_stop_ext_control(self, start):
#         request = Trigger.Request()
#         result = Trigger.Response()
#         if start:
#             self.node.get_logger().info('Attempting to start external control program')
#             cli_play = self.node.create_client(Trigger, 'dashboard_client/play')
#             Future = cli_play.call_async(request) 
#         else:
#             self.node.get_logger().info('Attempting to stop external control program')
#             cli_stop = self.node.create_client(Trigger, 'dashboard_client/stop')
#             Future = cli_stop.call_async(request)
#         counter = 0
#         while Future.done() != True: 
#                 time.sleep(0.1)
#                 counter += 1
#                 if counter > 10:
#                     self.node.get_logger().info('Received no response from ur5, assuming program could not be stopped or started..')
#                     break
#         try:
#             response = Future.result()
#             if response.success:
#                 self.node.get_logger().info(
#                         'message: %r' % (response.message))
#                 return True
#             else:
#                 self.node.get_logger().info(
#                     'error: %r' % (response.message))
#                 return False
#         except:
#             return False

#     def io_callback(self,msg):
#         pin = msg.digital_in_states[0].pin
#         if not self.prev_digital_in_state:
#             if msg.digital_in_states[0].state:
#                 if not self.pauze_state:
#                     self.node.get_logger().info('Demonstrator set to pauze mode..')
#                     self.pauze_state = True
#                     self.start_stop_ext_control(False)
#                 else:
#                     self.node.get_logger().info('Demonstrator is going to resume..')
#                     self.pauze_state = False
#                     self.start_stop_ext_control(True)
#         self.prev_digital_in_state = msg.digital_in_states[0].state
#         #self.node.get_logger().info('digital pin %i state: %i' % (pin, self.pauze_state))

#     # send request to service
#     def send_request(self, sensor_state):
#         state = String()
#         state.data = str(sensor_state)
#         req = WenglorSRV.Request()
#         req.sensor_state = state
#         self.future = self.cli.call_async(req)

#     # take single 2d scan from wenglor service
#     def request_scan(self):
#         result = WenglorSRV.Response()
#         if self.cli.service_is_ready():
#             self.send_request("measuring")
#             while self.future.done() != True: 
#                 time.sleep(0.001)
#                 #self.node.get_logger().info('waiting for wenglor service response')
#         #try to read response from service
#         try:
#             response = self.future.result()
#         except Exception as e:
#             self.node.get_logger().info(
#                 'Service call failed %r' % (e,))
#         else:
#             points = point_cloud2.read_points_numpy(response.pointcloud)
#             self.current_points = points
#             self.measure_points = points[points[:, 2] > 0]
#             self.scan_timestamp = response.pointcloud.header.stamp

#     # appends single scan to pointcloud
#     def append_scan_to_pointcloud(self):
#         # wait for transform to become available
#         while True:
#             if self.tfBuffer.can_transform('world', 'wenglor_mount', self.scan_timestamp):
#                 #self.node.get_logger().info('found tf!')
#                 break
#             else:
#                 time.sleep(0.01)
#         try:
#             transform = self.tfBuffer.lookup_transform('world', 'wenglor_mount', self.scan_timestamp)
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             self.node.get_logger().info('something went wrong with the transform while appending scan to pointcloud')
       
#         #append 2d scan to 3d pointcloud
#         q = [transform.transform.rotation.w,transform.transform.rotation.x,
#                     transform.transform.rotation.y,transform.transform.rotation.z]
#         R = self.quaternion_rotation_matrix(q)
#         points = [[point[0]*0.001, point[1],
#                      point[2]*0.001] for point in self.current_points]
#         points = points @ transpose(R)
#         self.curr_scan_tf = transform   
#         new_points_base = np.column_stack((points, self.current_points[:,3]))
#         new_points_base[:,0] += transform.transform.translation.x
#         new_points_base[:,1] += transform.transform.translation.y
#         new_points_base[:,2] += transform.transform.translation.z
#         self.current_points_base = np.concatenate((self.current_points_base,new_points_base), axis=0)
            
#     def publish_pointcloud(self):
#         header = Header()
#         header.frame_id = 'world'
#         header.stamp = self.node.get_clock().now().to_msg()
#         pointcloud2 = point_cloud2.create_cloud(header, self.fields, self.current_points_base)
#         self.pointcloud2_pub.publish(pointcloud2)

#     def send_move_request(self, speed, x, y, z, yaw, pitch, roll):
#         while self.pauze_state:
#             time.sleep(0.5)
#         Speed = Float32()
#         Speed.data = speed
#         pose = Pose()
#         position = Point()
#         orientation = Quaternion()
#         position.x = x
#         position.y = y
#         position.z = z
#         q = self.toQuaternion(yaw,pitch,roll)
#         orientation.w = q.w
#         orientation.x = q.x
#         orientation.y = q.y
#         orientation.z = q.z
#         pose.position = position
#         pose.orientation = orientation
#         #print('orientation:', orientation)
#         pose_velo = PoseAndVelo()
#         pose_velo.pose = pose
#         pose_velo.speed = Speed
#         self.move_request.publish(pose_velo)

#     def start_sensor(self):
#         self.send_request("start")
#         #time.sleep(0.1)

#     def stop_sensor(self):
#         self.send_request("stop")

#     def clear_pointcloud(self):
#         self.current_points_base = np.zeros((1,4))

#     def measure_and_publish_gap(self):
#         # init 
#         self.clear_pointcloud()
#         plotdata = Plotdata()
#         curr_pos = 0.0
#         prev_distance = 10
#         min_delta_distance = 0.002 # in meters
#         number_of_scans = 70
#         # take number of scans, 
#         # each scan is taken when the sensor has moved min_delta_distance from the previous scan location
#         for i in range(number_of_scans):
#             counter = 0
#             # wait for sensor to be moved a min distance
#             while True:
#                 try:
#                     transform = self.tfBuffer.lookup_transform('world', 'wenglor_mount', rclpy.time.Time())
#                 except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                     self.node.get_logger().info('something went wrong with the transform')
#                 else:
#                     break
#             #         #self.node.get_logger().info('pos_dif %f' % (prev_distance - transform.transform.translation.y) )
#             #         if (prev_distance - transform.transform.translation.y) < min_delta_distance and counter < 100:
#             #             time.sleep(0.002)
#             #             counter += 1
#             #         else:
#             #             break
#             # request scan from wenglor service    
#             try:
#                 self.request_scan()
#                 self.append_scan_to_pointcloud()
#             except:
#                 self.node.get_logger().info('No scan available!')
#                 continue

#             pos_dif =  self.curr_scan_tf.transform.translation.y - prev_distance
#             # self.node.get_logger().info('tf.y %f' % transform.transform.translation.y)
#             if i > 0:
#                 curr_pos += pos_dif*1000
#             prev_distance = self.curr_scan_tf.transform.translation.y

#             if i < 1:
#                 plotdata.gap_size.data = 0.0
#                 plotdata.x.data = 0.0
#                 self.plotdata_pub.publish(plotdata)

#             try:
#                 plotdata.gap_size.data = float(MeasureGap(self.node,self.measure_points))
#                 plotdata.x.data = curr_pos
#                 self.plotdata_pub.publish(plotdata)
                
#             except:
#                 self.node.get_logger().info('Gap measurement failed!')
#             self.node.get_logger().info('Current position: %f' % curr_pos)
#             #self.node.get_logger().info('y position: %f' % prev_distance)             
#             self.publish_pointcloud()

#     def sweep_scan(self):
#         self.send_move_request(0.2,-0.225, 0.1, 0.45,3.14, -1.57, 3.14)
#         self.node.get_logger().info('move request to init pose')
#         time.sleep(3)
#         # start with moving to left side of sweep scan move
#         self.send_move_request(0.1, -0.3834936490538903, -0.09999999999999996, 0.3, -0.34, -2, 0)
#         self.node.get_logger().info('move to left side of sweep path')
#         time.sleep(4)
#         # start with sweep scan
#         self.send_move_request(-0.1,-0.35, -0.1, 0.3,-0.34, -2, 0)
#         self.node.get_logger().info('move request for sweep path')
#         self.start_sensor()
#         self.clear_pointcloud()
#         for i in range(60):
#             try:
#                 self.request_scan()
#                 self.append_scan_to_pointcloud()
#                 self.publish_pointcloud()
#             except:
#                 self.node.get_logger().info('No scan available!')
#         self.stop_sensor()

#     def main_demo(self):
#         # send initial end-effector pose request (speed_scaling,x,y,z,yaw,pitch,roll)
#         self.send_move_request(0.2,-0.225, 0.1, 0.45,3.14, -1.57, 3.14)
#         self.node.get_logger().info('move request to init pose')
#         time.sleep(4)
#         if self.fancy_move:
#             self.send_move_request(0.0,-0.225, 0.1, 0.45,3.14, -1.57, 3.14)
#             self.node.get_logger().info('move request for circle path')
#             time.sleep(5)
#         # move to left side of plate
#         self.send_move_request(0.2,-0.3, 0.0, 0.28,3.14, -1.05, 3.14)
#         self.node.get_logger().info('move request to left side of beam')
#         time.sleep(2)
#         # start measurements and moving to the right
#         self.send_move_request(0.05,-0.3, 0.1, 0.28,3.14, -1.05, 3.14)
#         self.node.get_logger().info('move request to right side of beam')
#         self.start_sensor()
#         self.measure_and_publish_gap()
#         # stop taking measurements
#         self.stop_sensor()
#         if self.sweep_scan_enable:
#             self.sweep_scan()
    
#     #quaternion functions copied from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
#     #yaw (Z), pitch (Y), roll (X)
#     def toQuaternion(self,yaw, pitch, roll): 
    
#         #Abbreviations for the various angular functions
#         cy = np.cos(yaw * 0.5)
#         sy = np.sin(yaw * 0.5)
#         cp = np.cos(pitch * 0.5)
#         sp = np.sin(pitch * 0.5)
#         cr = np.cos(roll * 0.5)
#         sr = np.sin(roll * 0.5)

#         q = np.quaternion(1,0,0,0)
#         q.w = cr * cp * cy + sr * sp * sy
#         q.x = sr * cp * cy - cr * sp * sy
#         q.y = cr * sp * cy + sr * cp * sy
#         q.z = cr * cp * sy - sr * sp * cy

#         return q

#     def quaternion_rotation_matrix(self,Q):
#         # Extract the values from Q
#         q0 = Q[0]
#         q1 = Q[1]
#         q2 = Q[2]
#         q3 = Q[3]
        
#         # First row of the rotation matrix
#         r00 = 2 * (q0 * q0 + q1 * q1) - 1
#         r01 = 2 * (q1 * q2 - q0 * q3)
#         r02 = 2 * (q1 * q3 + q0 * q2)
        
#         # Second row of the rotation matrix
#         r10 = 2 * (q1 * q2 + q0 * q3)
#         r11 = 2 * (q0 * q0 + q2 * q2) - 1
#         r12 = 2 * (q2 * q3 - q0 * q1)
        
#         # Third row of the rotation matrix
#         r20 = 2 * (q1 * q3 - q0 * q2)
#         r21 = 2 * (q2 * q3 + q0 * q1)
#         r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
#         # 3x3 rotation matrix
#         rot_matrix = np.array([[r00, r01, r02],
#                             [r10, r11, r12],
#                             [r20, r21, r22]])
                                
#         return rot_matrix
