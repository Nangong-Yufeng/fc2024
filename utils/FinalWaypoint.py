import numpy as np
import mavros_msgs.srv 
import sys
import os
from pathlib import Path
FILE = os.getcwd()
ROOT = FILE # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
import mavros_msgs.msg
from utils.location import geodetic_to_enu, enu_to_geodetic
from utils.classes import WayPointShit
import mavros_msgs


def output_wp_to_file(path:str, req: mavros_msgs.srv.WaypointPush.Request | np.ndarray, type: int=0):
    if type == 0:
        with open(path, 'w') as f:
            f.write("QGC WPL 110\n")
            for i in range(len(req.waypoints)):
                f.write(f"{i}\t{0}\t{req.waypoints[i].frame}\t{req.waypoints[i].command}\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{req.waypoints[i].x_lat}\t{req.waypoints[i].y_long}\t{req.waypoints[i].z_alt}\t{1}\n")
    else:
        with open(path, 'w') as f:
            f.write("QGC WPL 110\n")
            f.write(f"{0}\t{0}\t{3}\t{5100}\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{req[0]}\t{req[1]}\t{req[2]}\t{0}\n")
            f.write(f"{1}\t{0}\t{3}\t{5100}\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{req[0]}\t{req[1]}\t{req[2]}\t{0}\n")


class DropWayPointGenA_V2(WayPointShit):
    def __init__(self, home, tg1_gps, tg2_gps, tg3_gps):
        self.home = home
        self.tg1_gps = tg1_gps
        self.tg2_gps = tg2_gps
        self.tg3_gps = tg3_gps
        self.aux_gps1 = [31.8371341, 106.9440193, 20]
        self.aux_gps2 = [31.8372289, 106.9441908, 20]
        self.aux_gps2 = [31.8372511, 106.9442593, 20]
        self.aux_gps2 = [31.85689370, 106.89632660, 20]
        
        self.tg1_enu = geodetic_to_enu(self.tg1_gps[0], self.tg1_gps[1], self.tg1_gps[2], self.home[0], self.home[1], self.home[2])
        self.tg2_enu = geodetic_to_enu(self.tg2_gps[0], self.tg2_gps[1], self.tg2_gps[2], self.home[0], self.home[1], self.home[2])
        self.tg3_enu = geodetic_to_enu(self.tg3_gps[0], self.tg3_gps[1], self.tg3_gps[2], self.home[0], self.home[1], self.home[2])
        self.aux_enu1 = geodetic_to_enu(self.aux_gps1[0], self.aux_gps1[1], self.aux_gps1[2], self.home[0], self.home[1], self.home[2])
        self.aux_enu2 = geodetic_to_enu(self.aux_gps2[0], self.aux_gps2[1], self.aux_gps2[2], self.home[0], self.home[1], self.home[2])
                
        self.det_ret, self.rally= self.gen_detect_waypoint()
        output_wp_to_file("waypoint/rally.txt", self.rally, type=1)
        output_wp_to_file("waypoint/way.txt", self.det_ret)
        
        self.tg1_ret = self.gen_drop_waypoint_v2(self.tg1_enu, 1)
        output_wp_to_file("waypoint/way1.txt", self.tg1_ret)
        self.tg2_ret = self.gen_drop_waypoint_v2(self.tg2_enu, 2)
        output_wp_to_file("waypoint/way2.txt", self.tg2_ret)
        self.tg3_ret = self.gen_drop_waypoint_v2(self.tg3_enu, 3)
        output_wp_to_file("waypoint/way3.txt", self.tg3_ret)

    # 比赛四个点天井点坐标生成侦察航线
    def gen_detect_waypoint(self):
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        
        # 平行跑道方向
        straight_dir = (np.array(self.aux_enu2) - np.array(self.tg1_enu)) / np.linalg.norm(np.array(self.aux_enu2) - np.array(self.tg1_enu))
        tag1 = np.array(self.tg3_enu) - straight_dir * 80.
        tag2 = np.array(self.tg3_enu)
        # 插入角方向
        slope_dir = (np.array(self.tg2_enu) - np.array(self.tg3_enu)) / np.linalg.norm(np.array(self.tg3_enu) - np.array(self.tg2_enu))
        tag3 = np.array(self.tg2_enu) + slope_dir * 10.
        # 垂直跑道方向
        vet_dir = gen_rotate(np.pi / 2) @ straight_dir[:2]
        vet_dir = np.array([vet_dir[0], vet_dir[1], 0])
        tag4 = tag3 + vet_dir * 68.
        tag5 = tag4 - straight_dir * 120.
        tag6 = np.array(self.tg1_enu) - straight_dir * 100.
        tag7 = np.array(self.tg1_enu)
        slope_dir = np.array(np.array(self.tg2_enu) - np.array(self.tg1_enu)) / np.linalg.norm(np.array(self.tg2_enu) - np.array(self.tg1_enu))
        tag8 = tag7 + slope_dir * 50.
        
        
        #TODO:盘旋点修改
        # competition version
        # rally = np.array(self.tg3_enu) - dir2 * 150
        rally = np.array(self.tg1_enu) + straight_dir * 120.
        rally = rally + vet_dir * 45
        rally[-1] = 35
        
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(tag1, tag2, 25.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(tag2, tag3, 25.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(tag3, tag4, np.pi, False, 18.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(tag4, tag5, 25.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(tag5, tag6, np.pi, False, 18.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(tag6, tag7, 25.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(tag7, tag8, 25.))
        
        return req, enu_to_geodetic(*rally, *self.home)
    
    # 从盘旋点开始出发
    # TODO: 针对性修改了一些东西
    def gen_drop_waypoint_v2(self, target: list, idx: int):
        h = target[2]
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        center = np.array(geodetic_to_enu(*self.rally, *self.home))[:2]
        target_2 = np.array(target)[:2]
        a = np.linalg.norm(target_2 - center)
        
        # 修改了盘旋半径
        b = 45.
        theta = np.arcsin(b/a)
        dir = ((center - target_2) / a)
        # TODO: change the neg
        dir = gen_rotate(-theta)@ dir
        if idx == 1 or idx == 4: st = 100 * dir + target_2 
        else: st = 100 * dir + target_2
        tmp_st = np.zeros(3)
        tmp_st[0] = st[0]
        tmp_st[1] = st[1]
        tmp_st[2] = target[2]
        if idx == 1 or idx == 4: ed = -40 * dir + target_2 
        else: ed = -40 * dir + target_2
        tmp_ed = np.zeros(3)
        tmp_ed[0] = ed[0]
        tmp_ed[1] = ed[1]
        tmp_ed[2] = target[2]
        req.waypoints.extend(self.generate_straight_line_waypoints(tmp_st, tmp_ed, increase=25.))
        return req

if __name__ == "__main__":
    # d = DropWayPointGenA([38.55836766, 115.14099924, 0], [38.55957692, 115.14290759, 15], [38.55971915, 115.14313070, 15], [38.55986327, 115.14298034, 15], [38.55970967, 115.14275965, 15])
    # e = DropWayPointGenA_V2([31.8371110, 106.9440972, 0], [31.8374309, 106.9440512, 20], [31.8371777, 106.9440978, 20], [31.8372746, 106.9438774, 20])
    e = DropWayPointGenA_V2([31.8371110, 106.9440972, 0], [31.85703750, 106.89632730, 20], [31.85689050, 106.89645300, 20], [31.85706590, 106.89654620, 20])
    
    # e = DropWayPointGenB_V2([38.557757, 115.140176, 0], [38.529368, 115.129752, 20], [38.529555, 115.129752, 20], [38.529555, 115.130000, 20], [38.529371, 115.129987, 20])

    # gen_sp_waypoint()
    # e = DropWayPointGenB([38.557757, 115.140176, 0], [38.557240, 115.138841, 15], [38.55734309, 115.13899788, 15], [38.557213, 115.139111, 15], [38.557113, 115.138961, 15])
    # e = DropWayPointGenA_V2([38.557757, 115.140176, 0], [38.557914, 115.139903, 20], [38.558036, 115.140067, 20], [38.558167, 115.139940, 20], [38.558048, 115.139759, 20])
    
