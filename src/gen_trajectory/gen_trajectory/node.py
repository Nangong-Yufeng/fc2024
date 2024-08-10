import mavros_msgs.msg
import rclpy
import numpy
import matplotlib.pyplot as plt
import mavros_msgs
import sensor_msgs.msg
import sensor_msgs
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from .location import geodetic_to_enu
 
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

path1="/home/joe/Desktop/trajectory_global.txt"
path2="/home/joe/Desktop/trajectory_enu.txt"


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.global_position_sub=self.create_subscription(sensor_msgs.msg.NavSatFix,"/mavros/global_position/global",self.global_position_cb,qos_profile)
        self.trajectory_global=[]
        self.trajectory_enu=[]
        self.home=None
        self.timer=self.create_timer(10,self.timer_cb)
        self.size=0
    def global_position_cb(self,global_position:sensor_msgs.msg.NavSatFix):
        self.trajectory_global.append([global_position.latitude,global_position.longitude,global_position.altitude])
    def output_trajectory(self):
        self.home=[*self.trajectory_global[0]]
        self.home[2]=0.
        print(self.home)
        for i in range(len(self.trajectory_global)):
            self.trajectory_enu.append(geodetic_to_enu(*self.trajectory_global[i],self.home[0],self.home[1],self.home[2]))
        with open(path1,'w') as f:
            for i in range(len(self.trajectory_global)):
                f.write(str(self.trajectory_global[i][0])+" "+str(self.trajectory_global[i][1])+" "+str(self.trajectory_global[i][2])+"\n")
        with open(path2,'w') as f:
            for i in range(len(self.trajectory_enu)):
                f.write(str(self.trajectory_enu[i][0])+" "+str(self.trajectory_enu[i][1])+" "+str(self.trajectory_enu[i][2])+"\n")
    def timer_cb(self):
        if len(self.trajectory_global) > self.size:
            print("没播放完...")
        else:
            self.output_trajectory()
            print("播放完毕")
        self.size=len(self.trajectory_global)

def main(args=None):
    rclpy.init(args=args)
    listener=Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()