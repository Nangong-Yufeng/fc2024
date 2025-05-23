import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import sys
import os
from pathlib import Path
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
import std_msgs.msg
from utils.classes import BaseNode, WayPointShit, TakeOffShit, ParameterShit, RallyPointShit, CallBackNode, State
from utils.location import geodetic_to_enu, enu_to_geodetic
# from utils.DropWayPointGen import DropWayPointGen
from utils.CompeitionWaypointA import DropWayPointGenA_V2, DropWayPointGenB_V2
import std_msgs
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
path1="/home/joe/Desktop/pose_velocity.txt"
wp_path = "waypoint/way"
'''
class PoseRelateNode(Node):

    def __init__(self):
        super().__init__('pose_relate_node')
        self.local_position_sub = self.create_subscription(PoseStamped,"/mavros/global_position/local",self.local_position_cb,qos_profile)
        self.local_velocity_sub = self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self.local_velocity_cb, qos_profile)
        self.data = []
        self.velocity = []
        self.timer = self.create_timer(10,self.timer_cb)
        self.size = 0
        
    def local_position_cb(self, local_position: PoseStamped) -> None:
        self.data.append([local_position.pose.position.x,local_position.pose.position.y,local_position.pose.position.z,
                          local_position.pose.orientation.x,local_position.pose.orientation.y,local_position.pose.orientation.z,local_position.pose.orientation.w,
                          self.velocity])

    def local_velocity_cb(self, local_velocity: TwistStamped) -> None:
        self.velocity = np.linalg.norm(np.array([local_velocity.twist.linear.x,local_velocity.twist.linear.y,local_velocity.twist.linear.z]))
        
    def output_file(self):
        with open(path1,'w') as f:
            for i in range(len(self.data)):
                f.write(str(self.data[i][0])+" "+str(self.data[i][1])+" "+str(self.data[i][2])+" "+
                        str(self.data[i][3])+" "+str(self.data[i][4])+" "+str(self.data[i][5])+" "+
                        str(self.data[i][6])+" "+str(self.data[i][7])+"\n")
                
    def timer_cb(self):
        if len(self.data) > self.size:
            print("没记录完...")
        else:
            self.output_file()
            print("记录完毕")
        self.size = len(self.data)     
'''

def init_wp_info():
    ret = []
    for i in range(1,5):
        with open(wp_path+str(i)+".txt") as f:
            lines = []
            for line in f.readlines():
                wp = line.split("\t")
                lines.append(wp)
            ret.append([[float(lines[2][8]), float(lines[2][9]), float(lines[2][10])], len(lines)-2])
    return ret

def init_rally_info():
    ret = []
    with open("waypoint/rally.txt") as f:
        lines = []
        for line in f.readlines():
            wp = line.split("\t")
            lines.append(wp)
        ret = [float(lines[2][8]), float(lines[2][9]), float(lines[2][10])]
    return ret

def check_vector(vec1, vec2):
    vec1 = np.array([vec1[0], vec1[1], 0])
    vec2 = np.array([vec2[0], vec2[1], 0])
    return np.cross(vec1, vec2)[-1] < 0 
    

class SwitchToAutoNode(WayPointShit, ParameterShit, RallyPointShit, TakeOffShit):
    def __init__(self):
        BaseNode.__init__(self)
        # 订阅标靶信息
        self.target_sub = self.create_subscription(std_msgs.msg.UInt32, "/vision/target_num", self.target_num_cb, qos_profile)
        
        self.final_drop_first_wp = []
        self.final_drop_wp_num = None
        self.detect_wp_num = 30
        self.final_target = None
        self.stage = -1
        
        # 关于集结点盘旋的判断
        # tmp = DropWayPointGenA_V2([38.557757, 115.140176, 0], [38.5570209, 115.1385889, 20], [38.5568987, 115.1384065, 20], [38.5567613, 115.1385473, 20], [38.556884, 115.138737, 20])
        self.tg_wp_info = init_wp_info()
        self.rally_point_gps = init_rally_info()
        self.rally_point_enu = []
        self.rally_hover_flg = False
        self.rally_hover_counter = 0
        self.rally_hover_log = False
        
        # 关于切换的判断
        self.switch_to_auto_log_flg = False
        self.wp_push_success_flg = 0
        self.wp_push_success_log_flg = False
        self.mission_fin = False
        
        # 计时器设置
        self.rally_hover_judge_timer = self.create_timer(0.1, self.rally_hover_judge_cb)
        self.gps_to_enu_timer = self.create_timer(0.1, self.gps_to_enu_cb)
        self.switch_to_auto_timer = self.create_timer(0.1, self.switch_to_auto_cb)
        self.wp_push_success_timer = self.create_timer(2, self.wp_push_success_cb)
        self.new_mission_fin_timer = self.create_timer(0.1, self.new_mission_fin_cb)
        self.param_chg_timer = self.create_timer(0.1, self.target_get_timer_cb)
        
        
    def wp_push_success_cb(self):
        if self.wp_push_success_flg: return
        
        if self.parameters_new_get_req == False and self.parameters_get_success == False:
            self.parameters_new_get_req = True
            self.get_parameters(["TARGET_AUTO"])
            
        elif self.parameters_new_get_req == False and self.parameters_get_success == True:
            self.parameters_get_success = False
            self.wp_push_success_flg = int(self.parameters_get_ret[0].double_value)
            if self.wp_push_success_flg == 1 and self.wp_push_success_log_flg == False:
                self.get_logger().info("检测到lua脚本上传航点成功!!")
                self.wp_push_success_log_flg = True
        
    
    def new_mission_fin_cb(self):
        if self.mission_fin == True: return
        if self.wp_push_success_flg == 0: return
        if self.new_mission == False: self.new_mission = True
        if self.current_reached_waypoint is None: return 
        if  self.current_reached_waypoint == self.final_drop_wp_num: 
            self.mission_fin = True
            self.get_logger().info("投弹已完成!!!")
    
    def switch_to_auto_cb(self):
        if self.state.mode == 'AUTO':
            self.switch_to_auto_log_flg = False
            return
        if len(self.final_drop_first_wp) == 0 or self.final_target == 0 or self.rally_hover_flg == False or self.wp_push_success_flg == 0 or self.mission_fin == True or self.stage != 4: return
        if self.switch_to_auto_log_flg == False:
            self.get_logger().info("即将切换到自动模式...")
            self.switch_to_auto_log_flg = True
        # 投影到平面
        tmp = geodetic_to_enu(self.final_drop_first_wp[0], self.final_drop_first_wp[1], self.final_drop_first_wp[2], *self.home)
        dir = self.rotate.apply([1, 0, 0])[:2]
        vec = (np.array(tmp) - np.array(self.local_position))[:2]
        dir = dir / np.linalg.norm(dir)
        vec = vec / np.linalg.norm(vec)
        theta = np.arccos(np.dot(vec, dir) / (np.linalg.norm(vec) * np.linalg.norm(dir))) * 180. / np.pi
        # TODO: tmp[0] 的正负需要根据情况进行修改! 
        if theta <= 40. and theta >= 30.:
            if not check_vector(dir, vec): return 
            self.stage = 3
        
    def gps_to_enu_cb(self):
        if self.home == None: return
        self.rally_point_enu = geodetic_to_enu(self.rally_point_gps[0], self.rally_point_gps[1], self.rally_point_gps[2], *self.home)
        
    def rally_hover_judge_cb(self):
        if self.state.mode != 'RTL' or self.home == None:
            self.rally_hover_flg = False 
            self.rally_hover_counter = 0
            self.rally_hover_log = False
            # TODO: 需要修改home点
            # self.home = [38.557757, 115.140176, 0]
            return
        elif self.rally_hover_counter >= 60:
            if self.stage == 0: self.stage = 1
            if self.rally_hover_log == False:
                print('\n')
                self.get_logger().info("---------------")
                self.get_logger().info("已经环绕集结点6秒, 进入投弹程序")
                self.get_logger().info("---------------")
                print('\n')
                self.rally_hover_log = True
            self.rally_hover_flg = True
        maxn = 60.
        dis = np.linalg.norm(np.array(self.local_position) - np.array(self.rally_point_enu))
    
        if dis < maxn: self.rally_hover_counter += 1

    def target_get_timer_cb(self):
        if self.stage == 1:
            if self.parameter_new_chg_req == False and self.parameter_chg_success == True:
                self.parameter_chg_success = False
                self.stage = 2
            elif self.parameter_new_chg_req == False and self.parameter_chg_success == False:
                self.parameter_new_chg_req = True
                self.chg_parameter("TARGET_NUM", float(self.final_target))
        
        elif self.stage == 2:
            if self.parameter_new_chg_req == False and self.parameter_chg_success == True:
                self.parameter_chg_success = False
                self.stage = 4
            elif self.parameter_new_chg_req == False and self.parameter_chg_success == False:
                self.parameter_new_chg_req = True
                self.chg_parameter("TARGET_GET", 1.0)
        
        elif self.stage == 3:
            if self.mode_new_set_req == False and self.mode_set_success == True:
                self.mode_set_success = False
                self.stage = 4
            elif self.mode_new_set_req == False and self.mode_set_success == False:
                self.mode_new_set_req = True
                self.set_mode('AUTO')
    
    def target_num_cb(self, target_num: std_msgs.msg.UInt32) -> None:
        if self.stage == -1:
            self.final_target = target_num.data
            self.get_logger().info("获取到目标标靶: " + str(self.final_target))
            self.final_drop_first_wp, self.final_drop_wp_num = self.tg_wp_info[self.final_target - 1]
            self.stage = 0

    
def main(args=None):
    rclpy.init(args=args)
    listener=SwitchToAutoNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()