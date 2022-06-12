# import setup_path
from .setup_path import SetupPath
import airsim
import math
import numpy as np
import time
import json
from numpy import linalg as LA
from abc import abstractmethod
from win32com.shell import shell, shellcon

############################
# Global Configurations
############################
enable_api_control = True  # True(Controlled by code) /False(Controlled by keyboard)
current_clock_speed = 1
no_control_interval_limit = 10000  # milli seconds
way_point_unit = 10  # (meter) DO NOT CHANGE THIS VALUE
forward_angles_cnt = 20
distance_way_points_cnt = 20
forward_obstacle_cnt = 20
opponent_cars_cnt = 20
############################


class NoControlError(Exception):
    pass


class DrivingController():

    def set_enable_api_control(self, enable_api_control_arg):
        global enable_api_control
        enable_api_control = enable_api_control_arg

    def __init__(self):
        print("[DrivingController] Welcome to Algorithm Contest! Driving will be started.")

        self.player_name = ""

        # check lap count
        self.map_num = "10"
        system_json = self.getJosnfile()

        self.getMapNum(system_json)
        self.getControlMode(system_json)
        self.getPlayerName(system_json)

        json_pos = self.getStartPos(system_json)
        self.opponent_cars = self.get_opponent_cars(system_json, json_pos)

        self.client = self.initialize_client()

        self.way_points, self.obstacle_points = self.load_track_info(self.client, json_pos)
        self.all_obstacles = DrivingUtil().get_all_obstacle_info(self.obstacle_points, self.way_points)

        self.backed_state = self.client.getCarState(self.player_name)
        self.client.setResetLocation(json_pos.x_val + 0.01, json_pos.y_val + 0.01, 0, self.player_name)

        self.control_interval = round(0.1 / current_clock_speed, 2)

        # road half width + car half width
        self.half_road_limit = self.client.getAlgoUserAPI().ac_road_width_half + 1.25


    def run(self):
        util = DrivingUtil()

        prev_state = self.client.getCarState(self.player_name)
        sensing_info = CarState(self.player_name)
        car_controls = airsim.CarControls()

        # freeze reset state
        self.freeze_time_stamp = 0
        prev_speed = 0.0

        lap_check_point_index = 0
        collision_time_stamp = 0
        prev_time_millis = 0

        game_start_flag = False
        game_stop_timer = 0
        return_code = 0

        current_lap = self.client.getAlgoUserAPI(self.player_name).ac_player_current_lap
        total_lap = self.client.getAlgoUserAPI(self.player_name).ac_max_lap

        while self.game_playing(self.client):
            car_state = self.client.getCarState(self.player_name)

            if prev_speed == car_state.speed:
                self.freeze_time_stamp += 1
                self.freeze_reset(self.client, self.freeze_time_stamp)
            else:
                self.freeze_time_stamp = 0
                prev_speed = car_state.speed

            if game_start_flag == True:
                if abs(car_state.speed) <= 1:
                    game_stop_timer += 1
                else:
                    game_stop_timer = 0
                if game_stop_timer > 300:
                    print("Player Stopped 30sec. Stop your game.")
                    return_code = 2
                    break
            else:
                game_start_flag = self.client.getAlgoAdminAPI().ac_RaceStart_YN

            for opp_car in self.opponent_cars:
                opp_car_state = self.client.getCarState(opp_car['car_name'])
                opp_car['car_state'] = opp_car_state

            prev, next = util.get_current_way_points(car_state, self.way_points, lap_check_point_index)

            lap_check_point_index = prev

            distance_from_center = util.get_distance_from_center(car_state, self.way_points, lap_check_point_index)
            right_of_center = util.is_right_of_center(car_state, self.way_points, lap_check_point_index)

            sensing_info.to_middle = distance_from_center * (1 if right_of_center else -1)

            collision_info = self.client.simGetCollisionInfo(self.player_name)
            sensing_info.collided = (collision_info.has_collided and collision_time_stamp < collision_info.time_stamp)
            collision_time_stamp = collision_info.time_stamp

            sensing_info.speed = util.get_speed(car_state)
            sensing_info.moving_forward = util.is_moving_forward(prev_state, car_state, self.way_points,
                                                                 lap_check_point_index)

            previous_x = prev_state.kinematics_estimated.position.x_val
            current_x = car_state.kinematics_estimated.position.x_val

            last_state = self.backed_state if previous_x == current_x else prev_state

            sensing_info.moving_angle = util.get_moving_angle(last_state, car_state, self.way_points,
                                                              lap_check_point_index)
            sensing_info.lap_progress = util.get_progress(car_state, self.way_points, lap_check_point_index,
                                                          current_lap, total_lap)

            if total_lap > 1 and sensing_info.lap_progress > 49.8 and sensing_info.lap_progress < 50.2:
                current_lap = self.client.getAlgoUserAPI(self.player_name).ac_player_current_lap
                sensing_info.lap_progress = util.get_progress(car_state, self.way_points, lap_check_point_index,
                                                              current_lap, total_lap)

            sensing_info.track_forward_angles = util.get_track_forward_angle(car_state, self.way_points,
                                                                             lap_check_point_index)
            sensing_info.track_forward_obstacles = util.get_track_forward_obstacle(car_state, self.way_points,
                                                                                   lap_check_point_index,
                                                                                   self.all_obstacles)
            sensing_info.opponent_cars_info = util.get_opponent_info(car_state, self.opponent_cars, self.way_points,
                                                                     lap_check_point_index)
            sensing_info.distance_to_way_points = util.get_distance_to_way_points(car_state, self.way_points,
                                                                                  lap_check_point_index)

            self.client.input_player_lap_progress(sensing_info.lap_progress, self.player_name)

            car_controls.steering = 0
            car_controls.throttle = 0
            car_controls.brake = 0

            car_controls = self.control_driving(car_controls, sensing_info)
            user_defined_brake = car_controls.brake

            self.set_gear(car_controls)

            ## Penalty ####################################
            if self.half_road_limit < distance_from_center:
                if abs(sensing_info.speed) > 40:
                    car_controls.brake = 0.9
                    #car_controls.throttle = car_controls.throttle * 0.5
                    # if car_controls.throttle > 0:
                    #    car_controls.is_manual_gear = True
                    #    car_controls.manual_gear = -1
                    #    car_controls.throttle = -1
                    # else:
                    #    car_controls.is_manual_gear = False
                    #    car_controls.throttle = 1
                    print("Penalty applied!!!")
                else:
                    car_controls.brake = user_defined_brake
            ###############################################
            self.client.setCarControls(car_controls, self.player_name)

            if round(previous_x, 4) != round(current_x, 4):
                self.backed_state = prev_state
            prev_state = car_state

            now_time_millis = self.get_current_milli_time()
            if prev_time_millis != 0 and now_time_millis - prev_time_millis >= no_control_interval_limit:
                raise NoControlError('No control was performed within 10000ms.')

            real_interval = self.control_interval - ((now_time_millis - prev_time_millis) / 1000)
            if real_interval < 0:
                real_interval = 0.05

            time.sleep(real_interval)
            prev_time_millis = self.get_current_milli_time()  # now_time_millis

        car_controls.throttle = 0
        car_controls.steering = 0.0
        car_controls.brake = 1
        self.client.setCarControls(car_controls, self.player_name)
        self.client.ac_rematch_check(self.player_name)

        return return_code

    def set_gear(self, car_controls):
        # Gear-R settings
        if car_controls.throttle < 0:
            car_controls.is_manual_gear = True
            car_controls.manual_gear = -1
        else:
            car_controls.is_manual_gear = False

    def game_playing(self, client):
        if client.getAlgoUserAPI(self.player_name).ac_RaceComplete_YN:
            return False
        return True

    def getPlayerName(self, json_data):
        # player_name_check = self.set_player_name()
        try:
            self.player_name = ""
            # sort : asc
            if ('Vehicles' in json_data):
                for key, value in sorted(json_data['Vehicles'].items()):
                    if self.player_name == "" and airsim.CarClient().isApiControlEnabled(key) == False:
                        self.player_name = key
                        break

            if "" == self.player_name:
                print("please check the settings.json and client player_name")
            print("[DrivingController] Player name : {}".format(self.player_name))
        except:
            pass

    def getMapNum(self, json_data):
        # check map num for lap count
        try:
            if ('Algo' in json_data):
                for key2, value2 in json_data['Algo'].items():
                    if (key2 == "Map"):
                        self.map_num = value2
                        print("[DrivingController] Map : {}".format(self.map_num))
                        break
        except:
            pass

    def getControlMode(self, json_data):
        # Control Mode
        try:
            if ('ControlMode' in json_data):
                json_control_mode = json_data['ControlMode']
                if json_control_mode == "Code":
                    global enable_api_control
                    enable_api_control = True
            print("[DrivingController] enable_api_control = {}".format(enable_api_control))
        except:
            pass

    def get_current_milli_time(self):
        return int(round(time.time() * 1000))

    def initialize_client(self):
        client = airsim.CarClient()
        client.confirmConnection()
        client.enableApiControl(enable_api_control, self.player_name)

        return client

    def load_track_info(self, client, json_pos):
        algo_apis = client.getAlgoUserAPI()
        way_points_raw = algo_apis.wayPoints
        obstacle_points_raw = algo_apis.ac_block_points
        way_points_revised = []
        obstacle_points_revised = []
        for x in range(0, len(way_points_raw)):
            if len(way_points_raw[x]):
                way_points_raw_trans = [way_points_raw[x][0] + (json_pos.x_val * -1),
                                        way_points_raw[x][1] + (json_pos.y_val * -1), 0.0]
                way_points_revised.append(way_points_raw_trans)
        for item in obstacle_points_raw:
            if len(item):
                obstacle_points_raw_trans = [item[0] + (json_pos.x_val * -1), item[1] + (json_pos.y_val * -1), 0.0]
                obstacle_points_revised.append(obstacle_points_raw_trans)

        if len(way_points_revised) == 188:
            del way_points_revised[187]

        return np.array(way_points_revised), np.array(obstacle_points_revised)

    def getJosnfile(self):
        path = shell.SHGetFolderPath(0, shellcon.CSIDL_PERSONAL, None, 0) + "\\Airsim\\settings.json"
        print("[DrivingController] Setting file position :", path)
        if path == "":
            path = "C:\\Users\\SDS\\Documents\\AirSim\\settings.json"
        with open(path, encoding='UTF-8') as json_file:
        #with open(path) as json_file:
            return json.load(json_file)

    def getStartPos(self, json_data):
        if self.player_name != "":
            json_car = json_data["Vehicles"][self.player_name]
            pos_setting = airsim.Vector3r(float(json_car["X"]), float(json_car["Y"]), 0)
        else:
            pos_setting = airsim.Vector3r(0, 0, 0)
        return pos_setting

    def get_opponent_cars(self, json_data, my_pos):
        try:
            cars = json_data["Vehicles"]
            if len(list(cars.keys())) <= 1:
                return []
        except:
            return []

        opponent_cars = []
        try:
            car_names = list(json_data["Vehicles"].keys())
            opp_car_names = list(filter(lambda name: name != self.player_name, car_names))

            start_index = 1 if len(car_names) == len(opp_car_names) else 0
            for idx in range(start_index, len(opp_car_names)):
                car_name = opp_car_names[idx]
                car_pos = json_data["Vehicles"][car_name]
                opponent_cars.append({"car_name": car_name,
                                      "x": car_pos["X"] - my_pos.x_val,
                                      "y": car_pos["Y"] - my_pos.y_val,
                                      "z": car_pos["Z"] - my_pos.z_val})
        except:
            print("cannot get the opponent cars. please check the settings.json")

        return opponent_cars

    def freeze_reset(self, client, time_stamp):
        if time_stamp == 10:
            self.freeze_time_stamp = 0
            client.reset()

    @abstractmethod
    def control_driving(self, car_controls, sensingInfo):
        raise NotImplementedError('Implement me in subclass')

    @abstractmethod
    def set_player_name(self):
        raise NotImplementedError('Implement me in subclass')


class DrivingUtil:

    def get_distance_from_center(self, car_state, way_points, check_point):
        prev, next = self.get_current_way_points(car_state, way_points, check_point)
        pd = car_state.kinematics_estimated.position
        car_pt = np.array([pd.x_val, pd.y_val, 0])
        dist = LA.norm(
            np.cross((car_pt - way_points[prev]), (way_points[next] - way_points[prev]))) / LA.norm(
            way_points[next] - way_points[prev])
        return round(dist, 2)

    def is_right_of_center(self, car_state, way_points, check_point):
        prev, next = self.get_current_way_points(car_state, way_points, check_point)
        pd = car_state.kinematics_estimated.position
        car_pt = np.array([pd.x_val, pd.y_val, 0])
        return self.get_cross_product_element_sign(car_pt, way_points[prev], way_points[next])

    def get_cross_product_element_sign(self, car_pt, wp_1, wp_2):
        v1 = car_pt - wp_1
        v2 = wp_2 - wp_1
        return np.cross(v1, v2)[2] < 0

    def get_current_obstacle_info_full_scan(self, obstacle_point, way_points):
        way_points_cnt = len(way_points)
        min_dist = 999999
        min_index = way_points_cnt - 1

        for x in range(0, way_points_cnt):
            calc_dist = LA.norm(way_points[x] - obstacle_point)
            if calc_dist < min_dist:
                min_dist = calc_dist
                min_index = x

        candidate_prev_index = self.get_next_N_waypoint_index(min_index, -1, way_points)
        candidate_next_index = self.get_next_N_waypoint_index(min_index, +1, way_points)

        dist_wp_prev = LA.norm(way_points[min_index] - way_points[candidate_prev_index])
        dist_wp_next = LA.norm(way_points[candidate_next_index] - way_points[min_index])

        prev_dist = LA.norm(obstacle_point - way_points[candidate_prev_index])
        next_dist = LA.norm(obstacle_point - way_points[candidate_next_index])

        if dist_wp_prev < 8 or dist_wp_next < 8 or dist_wp_prev > 12 or dist_wp_next > 12:
            if prev_dist * dist_wp_next >= next_dist * dist_wp_prev:
                wp_idx_1 = min_index
                wp_idx_2 = candidate_next_index
            else:
                wp_idx_1 = candidate_prev_index
                wp_idx_2 = min_index
        else:
            if prev_dist >= next_dist:
                wp_idx_1 = min_index
                wp_idx_2 = candidate_next_index
            else:
                wp_idx_1 = candidate_prev_index
                wp_idx_2 = min_index

        distance_unit = self.get_distance_unit(way_points, wp_idx_1, wp_idx_2)
        dist_from_wp1 = self.get_dist_to_intersection_point(obstacle_point, way_points, wp_idx_1, wp_idx_2)
        dist_from_wp1 = distance_unit if dist_from_wp1 > distance_unit else 0 if dist_from_wp1 < 0 else dist_from_wp1

        dist_from_wp1 = round(dist_from_wp1, 2)

        to_middle = LA.norm(
            np.cross((obstacle_point - way_points[wp_idx_1]),
                     (way_points[wp_idx_2] - way_points[wp_idx_1]))) / LA.norm(
            way_points[wp_idx_2] - way_points[wp_idx_1])

        if not self.get_cross_product_element_sign(obstacle_point, way_points[wp_idx_1], way_points[wp_idx_2]):
            to_middle = to_middle * -1

        return wp_idx_1, wp_idx_2, dist_from_wp1, to_middle

    def get_all_obstacle_info(self, obstacles, way_points):
        obstacle_sectors = []
        for x in obstacles:
            obstacle_sectors.append(self.get_current_obstacle_info_full_scan(x, way_points))
        return obstacle_sectors

    def get_dist_to_intersection_point(self, object_pt, way_points, prev, next):
        prev_pt = way_points[prev]
        next_pt = way_points[next]
        distance_unit = self.get_distance_unit(way_points, prev, next)

        cosine_val = np.dot(object_pt - prev_pt, next_pt - prev_pt) \
                     / LA.norm(next_pt - prev_pt) / LA.norm(object_pt - prev_pt)
        portion = LA.norm(object_pt - prev_pt) * cosine_val / LA.norm(next_pt - prev_pt)
        dist = distance_unit * portion

        return distance_unit if dist > distance_unit else 0 if dist < 0 else dist

    def get_current_way_points(self, car_state, way_points, check_point):
        pd = car_state.kinematics_estimated.position
        car_pt = np.array([pd.x_val, pd.y_val, 0])

        if check_point == False:
            first, last = -2, 10
        else:
            first = self.get_next_N_waypoint_index(check_point, -2, way_points)
            last = self.get_next_N_waypoint_index(check_point, 10, way_points)

        max_index = len(way_points)
        min_dist = 100000
        min_dist_idx = 0

        if first < last:
            for x in range(first, last):
                calc_dist = LA.norm(way_points[x] - car_pt)
                if min_dist > calc_dist:
                    min_dist = calc_dist
                    min_dist_idx = x
        else:
            for x in range(first, max_index):
                calc_dist = LA.norm(way_points[x] - car_pt)
                if min_dist > calc_dist:
                    min_dist = calc_dist
                    min_dist_idx = x

            for x in range(0, last):
                calc_dist = LA.norm(way_points[x] - car_pt)
                if min_dist > calc_dist:
                    min_dist = calc_dist
                    min_dist_idx = x

        min_dist_prev_idx = self.get_next_N_waypoint_index(min_dist_idx, -1, way_points)
        min_dist_next_idx = self.get_next_N_waypoint_index(min_dist_idx, 1, way_points)

        dist_wp_prev = LA.norm(way_points[min_dist_idx] - way_points[min_dist_prev_idx])
        dist_wp_next = LA.norm(way_points[min_dist_next_idx] - way_points[min_dist_idx])

        calc_dist_prev = LA.norm(way_points[min_dist_prev_idx] - car_pt)
        calc_dist_next = LA.norm(way_points[min_dist_next_idx] - car_pt)

        if dist_wp_prev < 8 or dist_wp_next < 8 or dist_wp_prev > 12 or dist_wp_next > 12:
            if calc_dist_prev * dist_wp_next < calc_dist_next * dist_wp_prev:
                return min_dist_prev_idx, min_dist_idx
            else:
                return min_dist_idx, min_dist_next_idx
        else:
            if calc_dist_prev < calc_dist_next:
                return min_dist_prev_idx, min_dist_idx
            else:
                return min_dist_idx, min_dist_next_idx

    def get_speed(self, car_state):
        return round(car_state.speed * 3.6, 2)

    def is_moving_forward(self, prev_state, current_state, way_points, check_point):
        prev, next = self.get_current_way_points(current_state, way_points, check_point)
        pd = current_state.kinematics_estimated.position
        car_pt = np.array([pd.x_val, pd.y_val, 0])
        pd = prev_state.kinematics_estimated.position
        prev_car_pt = np.array([pd.x_val, pd.y_val, 0])
        v1 = car_pt - prev_car_pt
        v2 = way_points[next] - way_points[prev]
        if np.dot(v1, v2) == 0 or (LA.norm(v1) * LA.norm(v2)) == 0:
            return True
        check_angle = np.dot(v1, v2) / (LA.norm(v1) * LA.norm(v2))
        if check_angle > 1:
            check_angle = 1
        elif check_angle < -1:
            check_angle = -1
        angle = math.acos(check_angle) * 180 / math.pi
        return -90 < angle < 90

    def get_moving_angle(self, prev_state, current_state, way_points, check_point):
        prev, next = self.get_current_way_points(current_state, way_points, check_point)
        pd = current_state.kinematics_estimated.position
        car_pt = np.array([pd.x_val, pd.y_val, 0])
        pd = prev_state.kinematics_estimated.position
        prev_car_pt = np.array([pd.x_val, pd.y_val, 0])

        v1 = car_pt - prev_car_pt
        v2 = way_points[next] - way_points[prev]

        if LA.norm(v1) == 0:
            return 0

        angle = self.get_v_angle(v1, v2)

        if not self.is_moving_forward(prev_state, current_state, way_points, check_point):
            if angle > 0:
                angle = angle - 180
            else:
                angle = angle + 180
        return round(angle, 1)

    def get_v_angle(self, v1, v2):
        check_angle = np.dot(v1, v2) / (LA.norm(v1) * LA.norm(v2))
        if check_angle > 1:
            check_angle = 1
        elif check_angle < -1:
            check_angle = -1
        angle = round(math.acos(check_angle) * 180 / math.pi, 1)
        if np.cross(v1, v2)[2] > 0:
            return angle * -1
        else:
            return angle

    driving_check = 0
    prev_progress = 0.0
    lap_check = False

    def get_progress(self, car_state, way_points, check_point, cur_lab, total_lap):
        prev, next = self.get_current_way_points(car_state, way_points, check_point)
        if next == 100:
            self.driving_check = cur_lab
        if self.lap_check is False and cur_lab == 2 and next == 0:
            self.lap_check = True
        if next == 0 and self.driving_check > 0:
            cur_lab = self.driving_check + 1
        curr = next + (len(way_points) - 1) * (cur_lab - 1)
        total = (len(way_points) - 1) * total_lap
        returnValue = round((curr / total) * 100, 2)

        if returnValue > self.prev_progress + 2:
            return self.prev_progress
        self.prev_progress = returnValue

        return returnValue

    def get_track_forward_angle(self, car_state, way_points, check_point):
        track_angle = []
        prev, next = self.get_current_way_points(car_state, way_points, check_point)
        v1 = way_points[next] - way_points[prev]
        for x in range(0, forward_angles_cnt):  # jmin7.park Increase the number of angle information
            t1 = self.get_next_N_waypoint_index(next, x, way_points)
            t2 = self.get_next_N_waypoint_index(next, x + 1, way_points)
            v2 = way_points[t1] - way_points[t2]
            angle = self.get_v_angle(v1, v2)
            if angle > 0:
                angle = 180 - angle
            else:
                angle = -180 - angle

            track_angle.append(round(angle))
        return track_angle

    def get_track_forward_obstacle(self, car_state, way_points, check_point, all_obstacles):
        track_obstacles = []
        prev, next = self.get_current_way_points(car_state, way_points, check_point)

        pd = car_state.kinematics_estimated.position
        car_pt = np.array([pd.x_val, pd.y_val, 0])

        distance_unit = self.get_distance_unit(way_points, prev, next)
        car_dist_from_prev = self.get_dist_to_intersection_point(car_pt, way_points, prev, next)
        car_dist_to_next = distance_unit - car_dist_from_prev

        for x in range(0, forward_obstacle_cnt):  # jmin7.park Increase the number of obstacle information
            t1 = self.get_next_N_waypoint_index(next, x, way_points)
            t2 = self.get_next_N_waypoint_index(next, x + 1, way_points)
            for obs in all_obstacles:
                if x == 0 and prev == obs[0] and next == obs[1]:
                    dist = round(obs[2] - car_dist_from_prev, 2)
                    if dist <= 200:
                        track_obstacles.append({"dist": dist, "to_middle": round(obs[3], 2)})
                elif t1 == obs[0] and t2 == obs[1]:
                    dist = round(x * way_point_unit + car_dist_to_next + obs[2], 2)
                    if dist <= 200:
                        track_obstacles.append({"dist": dist, "to_middle": round(obs[3], 2)})

        track_obstacles = sorted(track_obstacles, key=lambda obs: obs["dist"])
        return track_obstacles

    def get_range_indexes_array(self, from_index, to_index, max_index):
        range_indexes = []
        if from_index < to_index:
            for idx in range(from_index, to_index + 1):
                range_indexes.append(idx)
        else:
            for idx in range(from_index, max_index + 1):
                range_indexes.append(idx)
            for idx in range(0, to_index + 1):
                range_indexes.append(idx)
        return range_indexes

    normal_opp_car_pt = []

    def get_opponent_info(self, car_state, opponent_cars, way_points, check_point):
        prev, next = self.get_current_way_points(car_state, way_points, check_point)
        car_pt = self.get_point_of_car_state(car_state)

        distance_unit = self.get_distance_unit(way_points, prev, next)
        car_dist_from_prev = self.get_dist_to_intersection_point(car_pt, way_points, prev, next)
        car_dist_to_next = distance_unit - car_dist_from_prev

        from_point = self.get_prev_N_waypoint_index(prev, opponent_cars_cnt, way_points) # 20
        to_point = self.get_next_N_waypoint_index(next, opponent_cars_cnt, way_points) # 20
        range_indexes = self.get_range_indexes_array(from_point, to_point, len(way_points) - 1)

        last_wp_index = len(way_points) - 1
        opp_cars = []
        for opp_car in opponent_cars:
            opp_car_state = opp_car["car_state"]
            opp_car_pt = self.get_point_of_car_state(opp_car_state)

            # FILTERING
            if opp_car_pt[0] == 0 and opp_car_pt[1] == 0 and opp_car_pt[2] == 0:
                if not self.normal_opp_car_pt:
                    continue
                opp_car_pt = self.normal_opp_car_pt
            else:
                self.normal_opp_car_pt = [opp_car_pt[0], opp_car_pt[1], opp_car_pt[2]]

            opp_car_pt += np.array([opp_car["x"], opp_car["y"], opp_car["z"]])  # REQUIRED

            opp_prev, opp_next, opp_dist_from_prev, opp_to_middle = self.get_current_obstacle_info_full_scan(opp_car_pt,
                                                                                                             way_points)

            if (opp_prev not in range_indexes) or (opp_next not in range_indexes):
                continue

            opp_prev_index = range_indexes.index(opp_prev)
            opp_next_index = range_indexes.index(opp_next)
            if opp_prev_index > 20:
                dist = opp_dist_from_prev + (opp_prev_index - 21) * way_point_unit + car_dist_to_next
                for i in range(21, opp_prev_index):
                    if range_indexes[i] == last_wp_index:
                        dist = dist - way_point_unit + LA.norm(way_points[0] - way_points[last_wp_index])
                        break
            elif opp_prev_index == 20:
                dist = round(opp_dist_from_prev - car_dist_from_prev, 5)
            else:  # opp가 뒤
                opp_distance_unit = self.get_distance_unit(way_points, opp_prev, opp_next)
                opp_dist_to_next = opp_distance_unit - opp_dist_from_prev
                dist = opp_dist_to_next + (19 - opp_prev_index) * way_point_unit + car_dist_from_prev
                for i in range(opp_next_index, 20):
                    if range_indexes[i] == last_wp_index:
                        dist = dist - way_point_unit + LA.norm(way_points[0] - way_points[last_wp_index])
                        break
                dist *= -1

            if abs(dist) > 200:
                continue

            opp_cars.append({"car_name": opp_car["car_name"],
                             "dist": round(dist, 2),
                             "to_middle": round(opp_to_middle, 2),
                             "speed": round(opp_car_state.speed * 3.6, 2)})
            opp_cars = sorted(opp_cars, key=lambda opp_car: abs(opp_car["dist"]))
        return opp_cars

    def get_distance_to_way_points(self, car_state, way_points, check_point):
        prev, next = self.get_current_way_points(car_state, way_points, check_point)
        car_pt = self.get_point_of_car_state(car_state)

        dist_arr = []
        for x in range(0, distance_way_points_cnt):  # jmin.7park Increase the number of waypoint information
            wp_idx = self.get_next_N_waypoint_index(next, x, way_points)
            dist = LA.norm(way_points[wp_idx] - car_pt)
            dist = 0 if dist < 0 else dist
            dist_arr.append(round(dist, 2))
        return dist_arr

    def get_point_of_car_state(self, car_state):
        pd = car_state.kinematics_estimated.position
        car_pt = np.array([pd.x_val, pd.y_val, 0])
        return car_pt

    # privates
    def get_next_N_waypoint_index(self, current_index, n, way_points):
        next_index = current_index + n
        max_index = len(way_points) - 1

        if next_index > max_index:
            next_index = next_index - max_index - 1
        elif next_index < 0:
            next_index = next_index + max_index + 1
        return next_index

    # privates
    def get_prev_N_waypoint_index(self, current_index, n, way_points):
        prev_index = current_index - n
        if prev_index < 0:
            prev_index = len(way_points) + prev_index
        return prev_index

    # privates
    def get_distance_unit(self, way_points, prev, next):
        distance_unit = way_point_unit
        if next == 0:
            distance_unit = round(LA.norm(way_points[next] - way_points[prev]), 4)
        return distance_unit


class CarState:
    def __init__(self, name):
        self.__name = name

    collided = False
    collision_distance = 0
    speed = 0
    to_middle = 0
    moving_angle = 0

    moving_forward = True
    lap_progress = 0
    track_forward_angles = []
    track_forward_obstacles = []
    opponent_cars_info = []
    distance_to_way_points = []
