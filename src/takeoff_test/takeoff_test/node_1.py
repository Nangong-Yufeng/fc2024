import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import nav_msgs.msg
from rcl_interfaces.msg import SetParametersResult
import rclpy
import mavros_msgs
from rclpy.node import Node
import geometry_msgs
import sensor_msgs
import sensor_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

import numpy as np
import nav_msgs
import sys
sys.path.append("/home/joe/Desktop/NGYF_ws")
from utils.classes import BaseNode, WayPointShit, TakeOffShit, ParameterShit, RallyPointShit, CallBackNode
from utils.location import geodetic_to_enu, enu_to_geodetic
from utils.classes import State

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


square_mission_path = []


class MainNode(WayPointShit, TakeOffShit, ParameterShit, RallyPointShit):
    def __init__(self):
        BaseNode.__init__(self)
        self.timer = self.create_timer(0.1, self.timer_cb) 
        # 计时器
        self.offboard_setpoint_counter = 0
        self.param_name1 = "RTL_RADIUS"
        self.param_value1 = -30
        self.param_name2 = "RTL_ALTITUDE"
        self.param_value2 = 113
        self.param_name3 = "TARGET_NUM"
        self.param_value3 = 1
        self.param_name4 = "TARGET_GET"
        self.param_value4 = 1
        self.control_state = State.CHG_PARAM2
           
    def timer_cb(self):
        self.offboard_setpoint_counter += 1
        # if self.parameter_pull_success == False:
        #     self.pull_parameter(True)
        # else: self.parameter_new_chg_req = True
        if self.offboard_setpoint_counter <= 20: return
        # print (self.state)
        
        if self.control_state == State.PULL_PARAM:
            if self.parameter_new_pull_req == False and self.parameter_pull_success == True:
                self.parameter_pull_success = False
                self.control_state = State.CHG_PARAM
            elif self.parameter_new_pull_req == False and self.parameter_pull_success == False:
                self.parameter_new_pull_req = True
                self.pull_parameter(True)
        
        if self.control_state == State.CHG_PARAM:
            if self.parameters_new_set_req == False and self.parameters_set_success == True:
                self.parameters_set_success = False
                self.control_state = State.PUSH_RALLY
            elif self.parameters_new_set_req == False and self.parameters_set_success == False:
                self.parameters_new_set_req = True
                self.chg_parameters([self.param_name1, self.param_name2], [self.param_value1, self.param_value2])
                
        
        if self.control_state == State.CHG_PARAM2:
            if self.parameter_new_chg_req == False and self.parameter_chg_success == True:
                self.parameter_chg_success = False
                self.control_state = State.PUSH_RALLY
            elif self.parameter_new_chg_req == False and self.parameter_chg_success == False:
                self.parameter_new_chg_req = True
                self.chg_parameter(self.param_name1, self.param_value1)
        
        
         

class RandomWalkerNode(WayPointShit, TakeOffShit, ParameterShit, RallyPointShit):
    def __init__(self):
        BaseNode.__init__(self)
        self.home_tag = False
        self.takeoff = False
        self.mstate = 1
        self.timer = self.create_timer(1., self.timer_cb)
        
        self.diff_thread_start_time = 1
        self.diff_max = 0.
        self.max_real = None
        self.max_cal = None
        self.loiter_rallypoint_counter = 1 
        self.check_obit_rallypoint = False
        # 计时器
        self.offboard_setpoint_counter = 0
            
    
    def timer_cb(self):
        # print(self.state)
        if (self.home == None): return;
        
        if (len(self.local_position) == 3):
            # self.get_logger().info(f"local_position: {self.local_position}")
            tmp1 = np.array(geodetic_to_enu(*self.global_position,*self.home))
            tmp2 = np.array(self.local_position,dtype=np.float32)
            if (self.diff_max < np.linalg.norm(tmp1 - tmp2)):
                self.diff_max = np.linalg.norm(tmp1 - tmp2)
                self.max_real = tmp2
                self.max_cal = tmp1
            if (self.diff_thread_start_time == 20):
                self.get_logger().info("diff_max = {}, real_local_position = {}, cal_local_position = {}".format(self.diff_max, self.max_real, self.max_cal))
                self.diff_thread_start_time = 1
            self.diff_thread_start_time += 1
        
        if (self.mstate==1): 
            self.takeoff_process([0,100,40])
            self.new_mission = True
            self.current_waypoint_num = 1
            self.mstate = 2
            
        elif (self.mstate == 2): 
            self.arm()
            self.mstate = 3
            
        elif (self.mstate == 3):
            self.set_mode("AUTO")
            self.mstate = 4
            
        elif (self.mstate == 4):
            # self.get_logger().info(f"takeoff...{self.current_waypoint_num}, {self.current_reached_waypoint}")
            if (self.current_reached_waypoint == self.current_waypoint_num):
                self.new_mission = False
                self.get_logger().info("takeoff success!")
                self.mstate = 5
                
        elif (self.mstate == 5):
            maxn = 60.
            dis = np.linalg.norm(np.array(self.local_position) - np.array(geodetic_to_enu(*self.current_rallypoint, *self.home)))
            # self.get_logger().info(f"当前距离集结点的距离: {dis}")
            if (dis < maxn):self.loiter_rallypoint_counter += 1
            else: self.loiter_rallypoint_counter = 1
            if (self.loiter_rallypoint_counter > 10): self.mstate = 6 
            
        elif (self.mstate == 6):
            self.get_logger().info("loiter rallypoint...")
            self.mstate = 7
        """
        TODO: 距离集结点200-300米选取随机点
              随机选取直线或者曲线, 曲线的角度在120-180度之间
              随机选取点的高度在40-120米之间
              记录数据, 查找速度变化值
        """

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    # print("spin")
    node.destroy_node()
    rclpy.shutdown()
