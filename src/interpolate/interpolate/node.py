import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import multiprocessing
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from scipy import interpolate
import sys
sys.path.append("/home/joe/Desktop/NGYF_ws")
from utils.quaternion import Quaternion

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
path1="/home/joe/Desktop/position_pose_velocity.txt"


'''
TODO: 按时间戳取出对应的位置
      1. 创建客户端， 
'''
class CommunicateNode(Node):
    def __init__(self):
        pass
class InterpolateNode(Node):

    def __init__(self):
        super().__init__('interpolate_node')
        # self.local_position_sub = self.create_subscription(PoseStamped,"/mavros/local_position/pose",self.local_position_cb,qos_profile)
        # self.local_velocity_sub = self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self.local_velocity_cb, qos_profile)
        # self.global_position_sub = self.create_subscription(NavSatFix,"/mavros/global_position/global",self.global_position_cb, qos_profile)
        self.odom_sub = self.create_subscription(Odometry,"/mavros/global_position/local",self.odom_cb, qos_profile)
        self.error_maxn = 0
        
        self.data = []
        self.expand_data = []
        
        self.cubic = []
        self.quaternion = []
        self.hermite = []
        
        self.flag = False
        self.verify_data = []
        self.verify_spd = []
        
        
        self.spd = []
        self.errors = []
        self.fig = plt.figure(figsize=(5, 5))
        self.trajectory_ax = self.fig.add_subplot(111, projection='3d')
        self.tra_draw_timer = self.create_timer(0, self.draw)
        plt.ion()

        
    def draw(self):
        plt.cla()
        minn, maxn = 1e9, -1e9
        tmp = np.array([[0, 0, 0],[0, 0, 0]])
        if len(self.data) >= 40: 
            tmp = np.array(self.data)
            self.trajectory_ax.plot(tmp[-200:,0],tmp[-200:,1],tmp[-200:,2], color='black', label='ground_truth')
            self.trajectory_ax.scatter(tmp[-1,0], tmp[-1, 1], tmp[-1, 2], s=20, color='g')
            tmp = np.array(self.quaternion)
            self.trajectory_ax.plot(tmp[-1200:,0],tmp[-1200:,1],tmp[-1200:,2], label='quaternion')
            tmp = np.array(self.cubic)
            self.trajectory_ax.plot(tmp[-1200:,0],tmp[-1200:,1],tmp[-1200:,2], label='cubic')
            tmp = np.array(self.hermite)
            self.trajectory_ax.plot(tmp[-1200:,0],tmp[-1200:,1],tmp[-1200:,2], label='hermite')
            
        
        minn = min(tmp.min(), minn)
        maxn = max(tmp.max(), maxn)
        self.trajectory_ax.set_xlabel('East', fontsize=20)
        self.trajectory_ax.set_ylabel('North', fontsize=20)
        self.trajectory_ax.set_zlabel('Up', fontsize=20)
        plt.legend(loc='best')
        plt.title("trajectory interpolate", fontsize=20)
        # self.trajectory_ax.set_xlim([minn, maxn])
        # self.trajectory_ax.set_ylim([minn, maxn])
        # self.trajectory_ax.set_zlim([max(0, minn), maxn])
        plt.pause(0.01)
        
    def odom_cb(self, odom: Odometry) -> None:
        local_position = odom.pose
        velocity = np.linalg.norm(np.array([odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z]))
        self.data.append([local_position.pose.position.x,local_position.pose.position.y,local_position.pose.position.z,
                local_position.pose.orientation.x,local_position.pose.orientation.y,local_position.pose.orientation.z,local_position.pose.orientation.w,
                velocity, odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9])
        self.spd.append([odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z])
        if self.flag == False:
            self.verify_data.append(self.data[-1])
            self.verify_spd.append(self.spd[-1])
            self.flag = True
        else: self.flag = False
        
        # if len(self.data) < 2: return
        # self.quaternion_interpolate()
        if len(self.data) == 20: self.cubic_interpolate(True)
        elif len(self.data) > 20: 
            self.expand_data.append([self.data[-2][0], self.data[-2][1], self.data[-2][2]])

            hermite = self.hermite_interpolate()
            self.hermite.append([self.data[-2][0], self.data[-2][1], self.data[-2][2]])
            self.hermite.extend(hermite)
            self.hermite.append([self.data[-1][0], self.data[-1][1], self.data[-1][2]])
                
            
            
            cubic = self.cubic_interpolate(False)
            self.cubic.append([self.data[-2][0], self.data[-2][1], self.data[-2][2]])
            self.cubic.extend(cubic)
            self.cubic.append([self.data[-1][0], self.data[-1][1], self.data[-1][2]])
            
            
            
            self.quaternion.append([self.data[-2][0], self.data[-2][1], self.data[-2][2]])
            quaternion = self.quaternion_interpolate()
            self.quaternion.extend(quaternion)
            self.quaternion.append([self.data[-1][0], self.data[-1][1], self.data[-1][2]])
            
            error1 = np.array(self.get_error(hermite, self.data[-2][0:3], self.data[-1][0:3]))
            # error2 = np.array(self.get_error(quaternion, self.data[-2][0:3], self.data[-1][0:3]))
            
            # maxn1, mean1, std1 = error1.max(), error1.mean(), error1.std()
            # maxn2, mean2, std2 = error2.max(), error2.mean(), error2.std()
            # # print(maxn1, maxn2, mean1, mean2, std1, std2)
            # if min(maxn1, maxn2) > 3:
            #     if maxn1 < maxn2:
            #         self.expand_data.extend(cubic)
            #         self.errors.extend(error1)
            #     else:
            #         self.expand_data.extend(quaternion)
            #         self.errors.extend(error2)
            # elif max(maxn1, maxn2) > 5:
            #     if maxn1 < maxn2:
            #         self.expand_data.extend(cubic)
            #         self.errors.extend(error1)
            #     else:
            #         self.expand_data.extend(quaternion)
            #         self.errors.extend(error2)
            
            # elif mean1 + 3 * std1 < mean2 + 3 * std2 : 
            #     self.expand_data.extend(cubic)
            self.errors.extend(error1)
            # else: 
            #     self.expand_data.extend(quaternion)
            #     self.errors.extend(error2)
            self.expand_data.extend(hermite)

            
            self.expand_data.append([self.data[-1][0], self.data[-1][1], self.data[-1][2]]) 
            
            print(f"误差信息: {np.array(self.errors).max(), np.array(self.errors).mean(), np.array(self.errors).std()}")
    
    def get_error(self, data, last_pos, cur_pos):
        def getDis(p1, p2, p3):
            p1 = np.array(p1)
            p2 = np.array(p2)
            p3 = np.array(p3)
            vec1 = p1 - p2
            vec2 = p3 - p2
            return np.linalg.norm(np.cross(vec1, vec2)) / np.linalg.norm(vec2)  
        error = [] 
        for tt in data:
            # print (f"插值位置: {tt[0], tt[1], tt[2], getDis(last_pos, cur_pos, tt)}")
            # self.error_maxn = max(self.error_maxn, getDis(last_pos, cur_pos, tt))
            error.append(getDis(last_pos, cur_pos, tt))
        return error
        
    def hermite_interpolate(self, times = 5):
        recent_data = np.array(self.data)[-10:]
        recent_dif = np.array(self.spd)[-10:, 0:3]
        inc = (recent_data[-1][8] - recent_data[0][8]) / 9
        
        t = np.array([ i*inc for i in range(10)])
        
        
        fx = interpolate.CubicHermiteSpline(t, recent_data[:, 0], recent_dif[:, 0])
        fy = interpolate.CubicHermiteSpline(t, recent_data[:, 1], recent_dif[:, 1])   
        fz = interpolate.CubicHermiteSpline(t, recent_data[:, 2], recent_dif[:, 2])   
        
        i = -1
        delta = (t[i] - t[i-1]) / times
        inter_tmp = []
        
        # inter_tmp.append([recent_data[i-1][0], recent_data[i-1][1], recent_data[i-1][2]])
        for j in range(1, times):
            # z = f(recent_data[i-1][0] + delta_x * j, recent_data[i-1][1] + delta_y * j)[0]
            cur_t = t[i-1] + delta * j
            inter_tmp.append([fx(cur_t), fy(cur_t), fz(cur_t)])
        # self.expand_data.append([recent_data[-1][0], recent_data[-1][1], recent_data[-1][2]])
        return inter_tmp
                   
    def cubic_interpolate(self, flag, times = 5):
        recent_data = np.array(self.data)[-20:, 0:3]
        # print(recent_data)
        # f = interpolate.interp2d(recent_data[:, 0], recent_data[:, 1], recent_data[:, 2], kind='cubic')
        f = interpolate.bisplrep(recent_data[:, 0], recent_data[:, 1], recent_data[:, 2])
        if flag:
            for i in range(1,20):  
                delta_x = (recent_data[i][0] - recent_data[i-1][0]) / times
                delta_y = (recent_data[i][1] - recent_data[i-1][1]) / times
                self.expand_data.append([recent_data[i-1][0], recent_data[i-1][1], recent_data[i-1][2]])
                for j in range(1, times):
                    # z = f(recent_data[i-1][0] + delta_x * j, recent_data[i-1][1] + delta_y * j)[0]
                    z = interpolate.bisplev(recent_data[i-1][0] + delta_x * j, recent_data[i-1][1] + delta_y * j, f)
                    self.expand_data.append([recent_data[i-1][0] + delta_x * j, recent_data[i-1][1] + delta_y * j, z])
            self.expand_data.append([recent_data[-1][0], recent_data[-1][1], recent_data[-1][2]])
            return None
        else: 
            delta_x = (recent_data[-1][0] - recent_data[-2][0]) / times
            delta_y = (recent_data[-1][1] - recent_data[-2][1]) / times
            inter_tmp = []
            i = -1
            # inter_tmp.append([recent_data[i-1][0], recent_data[i-1][1], recent_data[i-1][2]])
            for j in range(1, times):
                # z = f(recent_data[i-1][0] + delta_x * j, recent_data[i-1][1] + delta_y * j)[0]
                z = interpolate.bisplev(recent_data[i-1][0] + delta_x * j, recent_data[i-1][1] + delta_y * j, f)
                inter_tmp.append([recent_data[i-1][0] + delta_x * j, recent_data[i-1][1] + delta_y * j, z])
            # self.expand_data.append([recent_data[-1][0], recent_data[-1][1], recent_data[-1][2]])
            return inter_tmp
            
                
    def get_rotate_mat(self, u:list, v:list):
        u = np.array(u)
        v = np.array(v)
        u = u / np.linalg.norm(u)
        v = v / np.linalg.norm(v)
        w = np.cross(u, v)
        w_norm = np.linalg.norm(w)
        I = np.identity(3)
        W = np.matrix([[0, -w[2], w[1]],
               [w[2], 0, -w[0]],
               [-w[1], w[0], 0]])
        R = I + np.sin(w_norm) * W + (1 - np.cos(w_norm)) * np.dot(W, W)
        return R           
    
    def quaternion_interpolate(self):
        inter = self.data[-1][8] - self.data[-2][8]
        mat = np.identity(3)
        last_rot = Quaternion(*self.data[-2][3:7])

        last_pos = self.data[-2][0:3]
        last_vel = self.data[-2][7]
        

        
        cur_rot = Quaternion(*self.data[-1][3:7])
        cur_pos = np.array([self.data[-1][0], self.data[-1][1], self.data[-1][2]])
        cur_vel = self.data[-1][7]
        
        tmp_r = None
        tmp_pos = []
        last_pos_cp = last_pos
        tmp_vel = None
        tmp_result = None
        
        times = 5
        
        tmp_r = (Quaternion.rotate_insert_slerp(last_rot.rot, cur_rot.rot, 1./times)[:-1])
        tmp_vel = (last_vel + cur_vel) / 2.
        
        # self.expand_data.append([*last_pos, *last_rot.rot.as_quat()])
        for r in tmp_r:
            vec = r.apply([1, 0, 0])
            vec = vec / np.linalg.norm(vec)
            delta = tmp_vel * (inter/times) * vec
            last_pos_cp = last_pos_cp + delta
            tmp_pos.append(last_pos_cp.tolist())
            # self.expand_data.append([*last_pos_cp, *r.as_quat()])
        # print (f"上一次位置:[ {last_pos[0], last_pos[1], last_pos[2]} ]")
        # print (f"当前位置:[ {cur_pos[0], cur_pos[1], cur_pos[2]} ]")
        
        return tmp_pos
        # time.sleep(1.)
        
    
    
    
                
        
def main(args=None):
    rclpy.init(args=args)
    listener=InterpolateNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()