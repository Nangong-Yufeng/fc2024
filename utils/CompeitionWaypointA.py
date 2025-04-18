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

"""
V1

class DropWayPointGenB(WayPointShit):
    def __init__(self, home, tg1_gps, tg2_gps, tg3_gps, tg4_gps):
        self.home = home
        self.tg1_gps = tg1_gps
        self.tg2_gps = tg2_gps
        self.tg3_gps = tg3_gps
        self.tg4_gps = tg4_gps
        
        self.tg1_enu = geodetic_to_enu(tg1_gps[0], tg1_gps[1], tg1_gps[2], home[0], home[1], home[2])
        self.tg2_enu = geodetic_to_enu(tg2_gps[0], tg2_gps[1], tg2_gps[2], home[0], home[1], home[2])
        self.tg3_enu = geodetic_to_enu(tg3_gps[0], tg3_gps[1], tg3_gps[2], home[0], home[1], home[2])
        self.tg4_enu = geodetic_to_enu(tg4_gps[0], tg4_gps[1], tg4_gps[2], home[0], home[1], home[2])
        
        self.rally = self.gen_rally_waypoint()
        self.output_wp_to_file("waypoint/rally.txt", self.rally, type=1)
        self.det_ret = self.gen_detect_waypoint()
        self.output_wp_to_file("waypoint/way.txt", self.det_ret)
        self.tg1_ret = self.gen_drop_waypoint_v2(self.tg1_enu, 1)
        self.output_wp_to_file("waypoint/way1.txt", self.tg1_ret)
        self.tg2_ret = self.gen_drop_waypoint_v2(self.tg2_enu, 2)
        self.output_wp_to_file("waypoint/way2.txt", self.tg2_ret)
        self.tg3_ret = self.gen_drop_waypoint_v2(self.tg3_enu, 3)
        self.output_wp_to_file("waypoint/way3.txt", self.tg3_ret)
        self.tg4_ret = self.gen_drop_waypoint_v2(self.tg4_enu, 4)
        self.output_wp_to_file("waypoint/way4.txt", self.tg4_ret)
        
        
    def get_subsidiary_point_loc(self, st:list, ed:list, dist1:float, dist2:float, dist3:float, angle:float) -> list:
        st = np.array(st)
        ed = np.array(ed)
        v = (ed - st) / np.linalg.norm(ed - st)
        ret1 = (ed + dist1 * v).tolist()
        ret2 = (st - dist2 * v).tolist()
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        v = -1*v;
        tmp = gen_rotate(angle) @ v[0:2]
        v = np.zeros(3)
        v[0:2] = tmp
        # v[-1] = ed[-1]
        ret3 = (ret2 + dist3*v).tolist()
        return ret1, ret2, ret3
        
    
    # 比赛四个点天井点坐标生成侦察航线
    def gen_detect_waypoint(self):
        subsidiary_point1, ret2, subsidiary_point2 = self.get_subsidiary_point_loc(self.tg2_enu, self.tg1_enu, 80, 30, 50, np.pi/6)
        subsidiary_point3, ret3, _ = self.get_subsidiary_point_loc(self.tg3_enu, self.tg4_enu, 30, 120, 50, np.pi/6)
        _, _, ret4 = self.get_subsidiary_point_loc(self.tg4_enu, self.tg3_enu, 0, 0, 60, -np.deg2rad(60))
        
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point1, ret2, increase=25.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(ret2, subsidiary_point2, increase=25.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(ret3, subsidiary_point2, (120. / 180.) * np.pi, False, 18.)[::-1][2:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(ret3, subsidiary_point3, increase=25.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point3, ret4, increase=25.)[1:-1])
        return req

    def output_wp_to_file(self, path:str, req: mavros_msgs.srv.WaypointPush.Request | np.ndarray, type: int=0):
        if type == 0:
            with open(path, 'w') as f:
                f.write("QGC WPL 110\n")
                for i in range(len(req.waypoints)):
                    f.write(f"{i}\t{0}\t{req.waypoints[i].frame}\t{req.waypoints[i].command}\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{req.waypoints[i].x_lat}\t{req.waypoints[i].y_long}\t{req.waypoints[i].z_alt}\t{1}\n")
        else:
            with open(path, 'w') as f:
                f.write("QGC WPL 110\n")
                f.write(f"{0}\t{0}\t{3}\t{5100}\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{req[0]}\t{req[1]}\t{req[2]}\t{0}\n")
    
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
        if idx == 1 or idx == 2: b = 60.
        else: b = 70.
        theta = np.arcsin(b/a)
        dir = ((center - target_2) / a)
        dir = gen_rotate(theta)@ dir
        if idx == 1 or idx == 4: st = 60 * dir + target_2 
        else: st = 80 * dir + target_2
        tmp_st = np.zeros(3)
        tmp_st[0] = st[0]
        tmp_st[1] = st[1]
        tmp_st[2] = target[2]
        if idx == 1 or idx == 4: ed = -60 * dir + target_2 
        else: ed = -40 * dir + target_2
        tmp_ed = np.zeros(3)
        tmp_ed[0] = ed[0]
        tmp_ed[1] = ed[1]
        tmp_ed[2] = target[2]
        req.waypoints.extend(self.generate_straight_line_waypoints(tmp_st, tmp_ed, increase=25.))
        return req
        
    # TODO: 针对性修改了一些东西
    def gen_rally_waypoint(self):
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        dir = np.array(self.tg4_enu)[:2] - np.array(self.tg3_enu)[:2]
        dir = dir / np.linalg.norm(dir)
        bot = np.array(self.tg4_enu)[:2] + dir * 100.
        vet = (np.array(self.tg3_enu)[:2] - np.array(self.tg4_enu)[:2]) / np.linalg.norm(np.array(self.tg3_enu)[:2] - np.array(self.tg4_enu)[:2])
        vet = gen_rotate(np.pi / 2) @ vet
        bot = bot + vet * 65.
        st = np.zeros(3)
        st[0] = bot[0]
        st[1] = bot[1]
        st[2] = 20
        st = enu_to_geodetic(*st, *self.home)
        return st

class DropWayPointGenA(WayPointShit):
    def __init__(self, home, tg1_gps, tg2_gps, tg3_gps, tg4_gps):
        self.home = home
        self.tg1_gps = tg1_gps
        self.tg2_gps = tg2_gps
        self.tg3_gps = tg3_gps
        self.tg4_gps = tg4_gps
        
        self.tg1_enu = geodetic_to_enu(tg1_gps[0], tg1_gps[1], tg1_gps[2], home[0], home[1], home[2])
        self.tg2_enu = geodetic_to_enu(tg2_gps[0], tg2_gps[1], tg2_gps[2], home[0], home[1], home[2])
        self.tg3_enu = geodetic_to_enu(tg3_gps[0], tg3_gps[1], tg3_gps[2], home[0], home[1], home[2])
        self.tg4_enu = geodetic_to_enu(tg4_gps[0], tg4_gps[1], tg4_gps[2], home[0], home[1], home[2])
        
        self.rally = self.gen_rally_waypoint()
        self.output_wp_to_file("waypoint/rally.txt", self.rally, type=1)
        self.det_ret = self.gen_detect_waypoint()
        self.output_wp_to_file("waypoint/way.txt", self.det_ret)
        self.tg1_ret = self.gen_drop_waypoint_v2(self.tg1_enu, 1)
        self.output_wp_to_file("waypoint/way1.txt", self.tg1_ret)
        self.tg2_ret = self.gen_drop_waypoint_v2(self.tg2_enu, 2)
        self.output_wp_to_file("waypoint/way2.txt", self.tg2_ret)
        self.tg3_ret = self.gen_drop_waypoint_v2(self.tg3_enu, 3)
        self.output_wp_to_file("waypoint/way3.txt", self.tg3_ret)
        self.tg4_ret = self.gen_drop_waypoint_v2(self.tg4_enu, 4)
        self.output_wp_to_file("waypoint/way4.txt", self.tg4_ret)
        
        
    def get_subsidiary_point_loc(self, st:list, ed:list, dist1:float, dist2:float, dist3:float, angle:float) -> list:
        st = np.array(st)
        ed = np.array(ed)
        v = (ed - st) / np.linalg.norm(ed - st)
        ret1 = (ed + dist1 * v).tolist()
        ret2 = (st - dist2 * v).tolist()
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        v = -1*v;
        tmp = gen_rotate(angle) @ v[0:2]
        v = np.zeros(3)
        v[0:2] = tmp
        # v[-1] = ed[-1]
        ret3 = (ret2 + dist3*v).tolist()
        return ret1, ret2, ret3
        
    
    # 比赛四个点天井点坐标生成侦察航线
    def gen_detect_waypoint(self):
        subsidiary_point1, ret2, subsidiary_point2 = self.get_subsidiary_point_loc(self.tg2_enu, self.tg1_enu, 80, 30, 50, -np.pi/6)
        subsidiary_point3, ret3, _ = self.get_subsidiary_point_loc(self.tg3_enu, self.tg4_enu, 30, 120, 50, np.pi/6)
        _, _, ret4 = self.get_subsidiary_point_loc(self.tg4_enu, self.tg3_enu, 0, 0, 60, np.deg2rad(60))
        
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point1, ret2, increase=25.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(ret2, subsidiary_point2, increase=25.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(subsidiary_point2, ret3, (120. / 180.) * np.pi, False, 18.)[1:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(ret3, subsidiary_point3, increase=25.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(subsidiary_point3, ret4, increase=25.)[1:-1])
        return req

    def output_wp_to_file(self, path:str, req: mavros_msgs.srv.WaypointPush.Request | np.ndarray, type: int=0):
        if type == 0:
            with open(path, 'w') as f:
                f.write("QGC WPL 110\n")
                for i in range(len(req.waypoints)):
                    f.write(f"{i}\t{0}\t{req.waypoints[i].frame}\t{req.waypoints[i].command}\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{req.waypoints[i].x_lat}\t{req.waypoints[i].y_long}\t{req.waypoints[i].z_alt}\t{1}\n")
        else:
            with open(path, 'w') as f:
                f.write("QGC WPL 110\n")
                f.write(f"{0}\t{0}\t{3}\t{5100}\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{req[0]}\t{req[1]}\t{req[2]}\t{0}\n")
    
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
        if idx == 1 or idx == 2: b = 60.
        else: b = 70.
        theta = np.arcsin(b/a)
        dir = ((center - target_2) / a)
        dir = gen_rotate(-theta)@ dir
        if idx == 1 or idx == 4: st = 60 * dir + target_2 
        else: st = 80 * dir + target_2
        tmp_st = np.zeros(3)
        tmp_st[0] = st[0]
        tmp_st[1] = st[1]
        tmp_st[2] = target[2]
        if idx == 1 or idx == 4: ed = -60 * dir + target_2 
        else: ed = -40 * dir + target_2
        tmp_ed = np.zeros(3)
        tmp_ed[0] = ed[0]
        tmp_ed[1] = ed[1]
        tmp_ed[2] = target[2]
        req.waypoints.extend(self.generate_straight_line_waypoints(tmp_st, tmp_ed, increase=25.))
        return req
        
    # TODO: 针对性修改了一些东西
    def gen_rally_waypoint(self):
        def gen_rotate(x):
            return np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
        dir = np.array(self.tg4_enu)[:2] - np.array(self.tg3_enu)[:2]
        dir = dir / np.linalg.norm(dir)
        bot = np.array(self.tg4_enu)[:2] + dir * 100.
        vet = (np.array(self.tg3_enu)[:2] - np.array(self.tg4_enu)[:2]) / np.linalg.norm(np.array(self.tg3_enu)[:2] - np.array(self.tg4_enu)[:2])
        vet = gen_rotate(-np.pi / 2) @ vet
        bot = bot + vet * 65.
        st = np.zeros(3)
        st[0] = bot[0]
        st[1] = bot[1]
        st[2] = 20
        st = enu_to_geodetic(*st, *self.home)
        return st

"""

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

class DropWayPointGenB_V2(WayPointShit):
    def __init__(self, home, tg1_gps, tg2_gps, tg3_gps, tg4_gps):
        self.home = home
        self.tg1_gps = tg1_gps
        self.tg2_gps = tg2_gps
        self.tg3_gps = tg3_gps
        self.tg4_gps = tg4_gps
        
        self.tg1_enu = geodetic_to_enu(self.tg1_gps[0], self.tg1_gps[1], self.tg1_gps[2], self.home[0], self.home[1], self.home[2])
        self.tg2_enu = geodetic_to_enu(self.tg2_gps[0], self.tg2_gps[1], self.tg2_gps[2], self.home[0], self.home[1], self.home[2])
        self.tg3_enu = geodetic_to_enu(self.tg3_gps[0], self.tg3_gps[1], self.tg3_gps[2], self.home[0], self.home[1], self.home[2])
        self.tg4_enu = geodetic_to_enu(self.tg4_gps[0], self.tg4_gps[1], self.tg4_gps[2], self.home[0], self.home[1], self.home[2])
        
        #  = self.gen_rally_waypoint()
        
        self.det_ret, self.rally= self.gen_detect_waypoint()
        output_wp_to_file("waypoint/rally.txt", self.rally, type=1)
        output_wp_to_file("waypoint/way.txt", self.det_ret)
        
        #TODO: 标靶点坐标
        # self.tg1_gps = [31.85703750, 106.89632730, 20]
        # self.tg2_gps = [31.8569133, 106.8964558, 20]
        # self.tg3_gps = [31.85706590, 106.89654620, 20]
        
        # self.tg1_gps = [31.8570113, 106.8963353, 20]
        # self.tg2_gps = [31.8568573, 106.8964678, 20]
        # self.tg3_gps = [31.8570296, 106.8965509, 20]
        
        self.tg1_gps = [31.8569931, 106.8963299, 20]
        self.tg2_gps = [31.8568514, 106.89645, 20]
        self.tg3_gps = [31.857014, 106.8965388, 20]
        
        self.tg1_enu = geodetic_to_enu(self.tg1_gps[0], self.tg1_gps[1], self.tg1_gps[2], self.home[0], self.home[1], self.home[2])
        self.tg2_enu = geodetic_to_enu(self.tg2_gps[0], self.tg2_gps[1], self.tg2_gps[2], self.home[0], self.home[1], self.home[2])
        self.tg3_enu = geodetic_to_enu(self.tg3_gps[0], self.tg3_gps[1], self.tg3_gps[2], self.home[0], self.home[1], self.home[2])
        
        
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
        dir1 = (np.array(self.tg2_enu) - np.array(self.tg1_enu)) / np.linalg.norm(np.array(self.tg2_enu) - np.array(self.tg1_enu))  # 2 -> 1
        tag1 = np.array(self.tg1_enu) - dir1 * 80.
        tag2 = np.array(self.tg2_enu) + dir1 * 10.
        vet = gen_rotate(np.pi / 2) @ dir1[:2]
        vet = np.array([vet[0], vet[1], 0])
        tag3 = tag2 + vet * 62.
        tag4 = tag3 - dir1 * 120.
        dir2 = (np.array(self.tg4_enu) - np.array(self.tg3_enu)) / np.linalg.norm(np.array(self.tg4_enu) - np.array(self.tg3_enu))  # 2 -> 1
        vet = gen_rotate(np.pi / 2) @ dir2[:2]
        vet = np.array([vet[0], vet[1], 0])
        rally = np.array(self.tg3_enu) - dir2 * 150
        rally = rally - vet * 40
        rally[-1] = 25
        # rot60 
        tag5 = np.array(self.tg4_enu) + dir2 * 80.
        tag6 = np.array(self.tg3_enu) - dir2 * 10.
        rot160 = gen_rotate(np.deg2rad(-140)).dot(dir2[:2])
        rot160 = np.array([rot160[0], rot160[1], 0])
        tag7 = tag6 + rot160 * 50
        
        req = mavros_msgs.srv.WaypointPush.Request()
        req.waypoints.append(self.generate_waypoint(0., 0., 0.))
        req.waypoints.extend(self.generate_straight_line_waypoints(tag1, tag2, increase=25.)[:-1])
        # req.waypoints.extend(self.generate_straight_line_waypoints(tag2, sub1, increase=50.))
        tag3[-1]=28
        req.waypoints.extend(self.generate_curve_line_waypoints(tag2, tag3, np.deg2rad(180), False, 18.)[:-1])
        tag4[-1]=28
        req.waypoints.extend(self.generate_straight_line_waypoints(tag3, tag4, 25.)[:-1])
        req.waypoints.extend(self.generate_curve_line_waypoints(tag4, tag5, np.pi, False, 18.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(tag5, tag6, 25.)[:-1])
        req.waypoints.extend(self.generate_straight_line_waypoints(tag6, tag7, 25.))
        
        
        
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
        dir = gen_rotate(-theta)@ dir
        if idx == 1 or idx == 4: st = 120 * dir + target_2 
        else: st = 100 * dir + target_2
        tmp_st = np.zeros(3)
        tmp_st[0] = st[0]
        tmp_st[1] = st[1]
        tmp_st[2] = target[2]
        if idx == 1 or idx == 4: ed = -60 * dir + target_2 
        else: ed = -60 * dir + target_2
        tmp_ed = np.zeros(3)
        tmp_ed[0] = ed[0]
        tmp_ed[1] = ed[1]
        tmp_ed[2] = target[2]
        req.waypoints.extend(self.generate_straight_line_waypoints(tmp_st, tmp_ed, increase=25.))
        return req
    
    

if __name__ == "__main__":
    # d = DropWayPointGenA([38.55836766, 115.14099924, 0], [38.55957692, 115.14290759, 15], [38.55971915, 115.14313070, 15], [38.55986327, 115.14298034, 15], [38.55970967, 115.14275965, 15])
    e = DropWayPointGenB_V2([31.8371110, 106.9440972, 0], [31.8570693, 106.8964873, 17], [31.8568984, 106.896497, 17], [31.8569025, 106.8963871, 17], [31.8570516, 106.8963691, 17], )
    # e = DropWayPointGenB_V2([38.557757, 115.140176, 0], [38.529368, 115.129752, 20], [38.529555, 115.129752, 20], [38.529555, 115.130000, 20], [38.529371, 115.129987, 20])

    # gen_sp_waypoint()
    # e = DropWayPointGenB([38.557757, 115.140176, 0], [38.557240, 115.138841, 15], [38.55734309, 115.13899788, 15], [38.557213, 115.139111, 15], [38.557113, 115.138961, 15])
    # e = DropWayPointGenA_V2([38.557757, 115.140176, 0], [38.557914, 115.139903, 20], [38.558036, 115.140067, 20], [38.558167, 115.139940, 20], [38.558048, 115.139759, 20])
    
