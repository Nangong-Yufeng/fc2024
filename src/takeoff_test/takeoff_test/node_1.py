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
import os
from pathlib import Path
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
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

from enum import Enum
class TestState(Enum):
    TEST_GET_HOME = 0
    TEST_PARAM_CHG = 1
    TETS_WP_PUSH_AND_CLEAR = 2
    TETS_RP_PUSH_AND_CLEAR = 3
    TETS_MODE_CHG = 4
    TEST_OVER = 5
    TEST_PARAM_GET = 6



class TestInfo:
    def __init__ (self, test_state:TestState, time:float | list):
        self.test_state = test_state
        self.time = time
        
class TestNode(WayPointShit, TakeOffShit, ParameterShit, RallyPointShit):
    def __init__(self):
        BaseNode.__init__(self, False)
        # 计时器
        self.test_param_list = [["RTL_RADIUS", -30], ["RTL_ALTITUDE", 113], ["WP_RADIUS", 1]]
        self.test_mode_list = ["AUTO", "STABILIZE", "RTL"]
        self.time_counter = 0
        self.test_state = TestState.TEST_PARAM_CHG
        self.control_state = State.CLEAR_WP
        self.test_wp = self.gen_test_wp()
        
        self.test_param_stage_round = 0
        self.test_wp_stage_round = 0
        self.test_rp_stage_round = 0
        self.test_mode_stage_round = 0
        self.round_limit = 30
        self.test_tot_round = 0
        
        self.test_consume_time_list = []
        
        self.test_timer = self.create_timer(0.01, self.test_timer_cb)
    
    def gen_test_wp(self):
        self.home = [22.59094024, 113.97535239, 0]
        self.get_logger().info("生成中...")
        req = mavros_msgs.srv.WaypointPush.Request()
        # start = geodetic_to_enu(22.59094024, 113.97535239, 120, *self.home)
        # end = geodetic_to_enu(22.58933996, 113.97537561, 120, *self.home)
        # req.waypoints.append(self.generate_waypoint(*start))
        # req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=20.))
        # self.waypoint_push(req)
        start = geodetic_to_enu(22.59094219, 113.97505543 , 120, *self.home)
        end = geodetic_to_enu(22.59004532, 113.97505278 , 120, *self.home)
        req.waypoints.append(self.generate_waypoint(*start))
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.))
        start = end
        end = geodetic_to_enu(22.590042, 113.975831 , 120, *self.home)
        req.waypoints.extend(self.generate_curve_line_waypoints(start, end, 180.*np.pi/180., False, increase=20.)[1:])
        # self.waypoint_push(req)
        start = geodetic_to_enu(22.590042, 113.975831 , 120, *self.home)
        end = geodetic_to_enu(22.590950, 113.975838, 110, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:-1])
            
        req = mavros_msgs.srv.WaypointPush.Request()
        start = geodetic_to_enu (22.590950, 113.975838, 110, *self.home)
        end = geodetic_to_enu (22.591887, 113.976163, 90, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:-1])
        
        start = geodetic_to_enu (22.591887,113.976163, 90, *self.home)
        end = geodetic_to_enu (22.591917, 113.975300, 75, *self.home)
        req.waypoints.extend(self.generate_curve_line_waypoints_radius(start, end, 48., False, 20.)[:-1])
        
        start = geodetic_to_enu (22.5919174000, 113.9752996000, 75, *self.home)
        end = geodetic_to_enu (22.591014, 113.975315, 50, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:-1])
        
        start = geodetic_to_enu (22.591014, 113.975315, 50, *self.home)
        end = geodetic_to_enu (22.590112, 113.975331, 50, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:])
        
        start = geodetic_to_enu (22.590112, 113.975331, 50, *self.home)
        end = geodetic_to_enu ( 22.5893637000, 113.9753522000, 90, *self.home)
        req.waypoints.extend(self.generate_straight_line_waypoints(start, end, increase=25.)[1:-1])
        
        start = geodetic_to_enu ( 22.5893637000, 113.9753522000, 90, *self.home)
        end = geodetic_to_enu (22.58975071, 113.97595540, 130, *self.home)
        req.waypoints.extend(self.generate_curve_line_waypoints_radius(start, end, 40., False, 20)[:])
        self.home = None
        return req
    
    def output_test_info(self, msg:str, test_info:TestInfo):
        print('\n')
        self.get_logger().info("--------------------")
        self.get_logger().info(msg)
        if isinstance(test_info.time, float): self.get_logger().info(f"当前测试状态: {test_info.test_state.name}，耗时: {test_info.time}s")
        else: 
            
            times = np.array(test_info.time)
            self.get_logger().info(f"当前测试状态: {test_info.test_state.name}，平均耗时: {np.mean(times)}s，最大耗时: {np.max(times)}s，最小耗时: {np.min(times)}s")
        self.get_logger().info("--------------------")
        
    
    def test_timer_cb(self):
        if self.test_state == TestState.TEST_GET_HOME:
            self.time_counter += 1
            if self.home == None: return
            else: 
                self.test_state = TestState.TEST_PARAM_CHG
                self.output_test_info("获取home成功", TestInfo(TestState.TEST_GET_HOME, self.time_counter*0.01))
                self.time_counter = 0
        
        if self.test_state == TestState.TEST_PARAM_CHG:
            self.time_counter += 1
            param_idx = self.test_param_stage_round % len(self.test_param_list)
            if self.parameter_new_chg_req == False and self.parameter_chg_success == True:
                self.parameter_chg_success = False
                self.test_param_stage_round += 1
                if self.test_param_stage_round == self.round_limit:
                    self.test_state = TestState.TETS_WP_PUSH_AND_CLEAR
                    self.control_state = State.CLEAR_WP
                    self.test_consume_time_list.append(self.time_counter*0.01)
                    self.time_counter = 0
                    self.output_test_info(f"参数设置共{self.round_limit}次", TestInfo(TestState.TEST_PARAM_CHG, self.test_consume_time_list))
                    self.test_consume_time_list = []
                    self.test_param_stage_round = 0
                else:
                    
                    self.test_consume_time_list.append(self.time_counter*0.01)
                    self.time_counter = 0 
                    self.parameter_new_chg_req = True
                    self.chg_parameter(self.test_param_list[param_idx][0], self.test_param_list[param_idx][1])
            elif self.parameter_new_chg_req == False and self.parameter_chg_success == False:
                self.parameter_new_chg_req = True
                self.chg_parameter(self.test_param_list[param_idx][0], self.test_param_list[param_idx][1])
        
        if self.test_state == TestState.TETS_WP_PUSH_AND_CLEAR:
            self.time_counter += 1        
            if self.control_state == State.CLEAR_WP:
                if self.waypoint_new_clear_req == False and self.waypoint_clear_success == True:
                    self.waypoint_clear_success == False
                    self.control_state = State.PUSH_WP
                elif self.waypoint_new_clear_req == False and self.waypoint_clear_success == False:
                    self.waypoint_new_clear_req = True
                    self.waypoint_clear()
            
            elif self.control_state == State.PUSH_WP:
                if self.waypoint_new_push_req == False and self.waypoint_push_success == True:
                    self.waypoint_push_success == False
                    self.test_wp_stage_round += 1
                    if self.test_wp_stage_round == self.round_limit:
                        self.test_state = TestState.TETS_RP_PUSH_AND_CLEAR
                        self.control_state = State.CLEAR_RALLY
                        self.test_consume_time_list.append(self.time_counter*0.01)
                        self.time_counter = 0
                        self.output_test_info(f"航点上传测试共{self.round_limit}次，航点数为{len(self.test_wp.waypoints)}", TestInfo(TestState.TETS_WP_PUSH_AND_CLEAR, self.test_consume_time_list))
                        self.test_consume_time_list = []
                        self.test_wp_stage_round = 0
                    else:
                        self.test_consume_time_list.append(self.time_counter*0.01)
                        self.control_state = State.CLEAR_WP
                        self.time_counter = 0
                elif self.waypoint_new_push_req == False and self.waypoint_push_success == False:
                    self.waypoint_new_push_req = True
                    self.waypoint_push(self.test_wp)
            
        
        if self.test_state == TestState.TETS_RP_PUSH_AND_CLEAR:
            self.time_counter += 1
            if self.control_state == State.CLEAR_RALLY:
                if self.rallypoint_new_clear_req == False and self.rallypoint_clear_success == True:
                    self.rallypoint_clear_success == False
                    self.control_state = State.PUSH_RALLY
                elif self.rallypoint_new_clear_req == False and self.rallypoint_clear_success == False:
                    self.rallypoint_new_clear_req = True
                    self.rallypoint_clear()
            
            elif self.control_state == State.PUSH_RALLY:
                # if self.state.mode != "AUTO": return
                if self.rallypoint_new_push_req == False and self.rallypoint_push_success == True:
                    self.rallypoint_push_success == False
                    self.test_rp_stage_round += 1
                    if self.test_rp_stage_round == self.round_limit:
                        self.test_state = TestState.TETS_MODE_CHG
                        self.control_state = State.CLEAR_WP
                        self.test_consume_time_list.append(self.time_counter*0.01)
                        self.time_counter = 0
                        self.output_test_info(f"集结点上传测试共{self.round_limit}次", TestInfo(TestState.TETS_RP_PUSH_AND_CLEAR, self.test_consume_time_list))
                        self.test_consume_time_list = []
                        self.test_rp_stage_round = 0
                    else:
                        self.test_consume_time_list.append(self.time_counter*0.01)
                        self.control_state = State.CLEAR_RALLY
                        self.time_counter = 0
                elif self.rallypoint_new_push_req == False and self.rallypoint_push_success == False:
                    self.rallypoint_new_push_req = True
                    self.rallypoint_push([22.58997037, 113.97536734, 120.])
        
        
        if self.test_state == TestState.TETS_MODE_CHG:
            self.time_counter += 1
            mode_idx = self.test_mode_stage_round % len(self.test_mode_list)
            if self.mode_new_set_req == False and self.mode_set_success == True:
                self.mode_set_success = False
                self.test_mode_stage_round += 1
                if self.test_mode_stage_round == self.round_limit:
                    self.test_state = TestState.TEST_PARAM_GET
                    self.test_consume_time_list.append(self.time_counter*0.01)
                    self.time_counter = 0
                    self.output_test_info(f"模式设置共{self.round_limit}次", TestInfo(TestState.TETS_MODE_CHG, self.test_consume_time_list))
                    self.test_consume_time_list = []
                    self.test_mode_stage_round = 0
                else:
                    self.test_consume_time_list.append(self.time_counter*0.01)
                    self.time_counter = 0 
                    self.mode_new_set_req = True
                    self.set_mode(self.test_mode_list[mode_idx])
            elif self.mode_new_set_req== False and self.mode_set_success == False:
                self.mode_new_set_req = True
                self.set_mode(self.test_mode_list[mode_idx])
        
        if self.test_state == TestState.TEST_PARAM_GET:
            self.time_counter += 1
            param_idx = self.test_param_stage_round % len(self.test_param_list)
            if self.parameters_new_get_req == False and self.parameters_get_success == True:
                self.parameters_get_success = False
                self.test_param_stage_round += 1
                if self.test_param_stage_round == self.round_limit:
                    self.test_state = TestState.TEST_OVER
                    self.control_state = State.CLEAR_WP
                    self.test_consume_time_list.append(self.time_counter*0.01)
                    self.time_counter = 0
                    self.output_test_info(f"参数获取共{self.round_limit}次", TestInfo(TestState.TEST_PARAM_GET, self.test_consume_time_list))
                    self.test_consume_time_list = []
                    self.test_param_stage_round = 0
                else:
                    self.test_consume_time_list.append(self.time_counter*0.01)
                    self.time_counter = 0 
                    self.parameters_new_get_req = True
                    self.get_parameters([self.test_param_list[param_idx][0]])
            elif self.parameters_new_get_req == False and self.parameters_get_success == False:
                self.parameters_new_get_req = True
                self.get_parameters([self.test_param_list[param_idx][0]])

        
        if self.test_state == TestState.TEST_OVER:
            if self.control_state == State.CLEAR_WP:
                if self.waypoint_new_clear_req == False and self.waypoint_clear_success == True:
                    self.waypoint_clear_success == False
                    self.control_state = State.PUSH_WP
                elif self.waypoint_new_clear_req == False and self.waypoint_clear_success == False:
                    self.waypoint_new_clear_req = True
                    self.waypoint_clear()
        
        
        
class MainNode(WayPointShit, TakeOffShit, ParameterShit, RallyPointShit):
    def __init__(self):
        BaseNode.__init__(self)
        self.timer = self.create_timer(0.1, self.timer_cb) 
        # 计时器
        self.offboard_setpoint_counter = 0
        self.param_name1 = "TARGET_GET"
        self.param_value1 = 1
        self.param_name2 = "RTL_ALTITUDE"
        self.param_value2 = 113
        self.param_name3 = "TARGET_NUM"
        self.param_value3 = 1
        self.param_name4 = "TARGET_GET"
        self.param_value4 = 1
        self.control_state = State.CHG_PARAM2
        
        
    # TODO: 写一个飞控测试流程
    def timer_cb(self):
        self.offboard_setpoint_counter += 1
        # if self.parameter_pull_success == False:
        #     self.pull_parameter(True)
        # else: self.parameter_new_chg_req = True
        if self.offboard_setpoint_counter <= 20: return
        
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
                self.control_state = State.CLEAR_RALLY
            elif self.parameter_new_chg_req == False and self.parameter_chg_success == False:
                self.parameter_new_chg_req = True
                self.chg_parameter(self.param_name1, self.param_value1)

        if self.control_state == State.CLEAR_WP:
            if self.waypoint_new_clear_req == False and self.waypoint_clear_success == True:
                self.waypoint_clear_success == False
                self.control_state = State.PUSH_WP
            elif self.waypoint_new_clear_req == False and self.waypoint_clear_success == False:
                self.waypoint_new_clear_req = True
                self.waypoint_clear()
        

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
