from DrivingInterface.drive_controller import DrivingController
import math

before = 0

class DrivingClient(DrivingController):

    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False

        # api or keyboard
        self.enable_api_control = True # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        self.accident_step = 0
        self.uturn_step = 0
        self.uturn_count = 0

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        half_load_width = self.half_road_limit - 2.7
        car_controls.throttle = 1
        car_controls.brake = 0


        # way points 좌표 시작
        middle = sensing_info.to_middle
        spd = sensing_info.speed

        # 오른쪽인가 왼쪽인가
        plag = 1 if middle >= 0 else -1


        points = [middle * plag,] + sensing_info.distance_to_way_points
        angles = [0,] + [angle * plag for angle in sensing_info.track_forward_angles]
        bo = [90, ]
        ts = []
        for i in range(20):
            C = 180 - bo[i] - (angles[i+1] - angles[i])
            temp = points[i] * math.sin(C * math.pi / 180) / points[i+1]
            A =  math.asin(temp if abs(temp) <= 1 else int(temp)) * 180 / math.pi
            bo.append(A)
            ts.append(180 - C - A)

        # way point 좌표
        ways = []
        for j in range(20): 
            ways.append([points[j+1] * math.sin(sum(ts[:j+1]) * math.pi / 180), - points[j+1] * plag * math.cos(sum(ts[:j+1]) * math.pi / 180)])


        # 장애물 좌표
        obs = []
        near = abs(points[0] * math.cos((90 - angles[1]) * math.pi / 180)) + points[1] * math.cos(bo[1] * math.pi / 180)
        for obj in sensing_info.track_forward_obstacles:
            d, m = obj['dist'] - near, obj['to_middle']
            if d <= 0:
                n, k = -1, obj['dist']
                ang = (90 - angles[n+1] * plag) * math.pi / 180
                obs.append([k * math.sin(ang) - m * math.cos(ang), -middle + k * math.cos(ang) + m * math.sin(ang), obj['dist'], obj['to_middle']])
    
            else:
                n, k = int(d // 10), d % 10
                if n+2 > 10:
                    break
                ang = (90 - angles[n+1] * plag) * math.pi / 180
                obs.append([ways[n][0] + k * math.sin(ang) - m * math.cos(ang), ways[n][1] + k * math.cos(ang) + m * math.sin(ang), obj['dist'], obj['to_middle']])



        ################################## 삽입 부분 #########################################

        # 인식거리 설정
        ob_start = 0
        ob_end = 150

        # 범위 
        ob_line = [round(i * 0.1, 1) for i in range(-int(half_load_width)*10, int(half_load_width)*10)]
        ob_line2 = []
        temp_dist = 0
        first_ob = 0

        # 인식거리 안의 장애물 등록
        # if sensing_info.track_forward_obstacles and sensing_info.track_forward_obstacles[0]['dist'] < 150:
        #     for obj in sensing_info.track_forward_obstacles:
        #         ob_dist, ob_middle = obj['dist'], obj['to_middle']
                
        #         if not first_ob:
        #             first_ob = ob_dist
        #         if temp_dist and abs(ob_dist - temp_dist) > 40:
        #             break

        #         if ob_start <= ob_dist <= ob_end:
        #             ped = 2.5
        #             if abs(angles[int(ob_dist/10)]) > 5:
        #                 ped = 3.5
        #             else:
        #                 ped = 2.25
        #             ob_line = [i for i in ob_line if not ob_middle-ped <= i <= ob_middle+ped]                
        #             temp_dist = ob_dist
        #             # print(ped)
                    
        #         if not ob_line:
        #             ob_line = ob_line2[:]
        #             break
        #         else:
        #             ob_line2 = ob_line[:]
        


        # target = min(ob_line,key = lambda x : abs(x - middle))
        # p = - (middle - target) * 0.1
        # i = p ** 2 * 0.05 if p >= 0 else - p ** 2 * 0.05 

        # middle_add = 0.5 * p + 0.4 * i


        ################################## 삽입 부분 #########################################
        

        # 주행 코드

        if spd < 50 and sensing_info.lap_progress > 1:
            tg = 0
        elif spd < 120:
            tg = 1
        elif spd < 180:
            tg = 2
        else:
            tg = 4
        angles = sensing_info.track_forward_angles
        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        if (angles[tg]) < 45:
            if len(sensing_info.track_forward_obstacles) == 0 and spd < 160:
                car_controls.steering = (angles[tg] - sensing_info.moving_angle) / 90 - middle / 80
            else:
                if spd < 70:
                    set_steering = (angles[tg] - sensing_info.moving_angle) / 60
                else:
                    set_steering = (angles[tg] - sensing_info.moving_angle) / 90
                car_controls.steering = set_steering
        else:
            # print(angles[tg])
            k = spd * 5 / 18
            if angles[tg] < 0:
                r = self.half_road_limit - 1.25 + middle
                beta = - math.pi * k * 0.1 / r
                car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if angles[tg] > -60 else -1
            else:    
                r = self.half_road_limit - 1.25 - middle
                beta = math.pi * k * 0.1 / r
                car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if angles[tg] < 60 else 1





        if abs(angles[int(spd//20)]) > 40 and spd > 90:
            car_controls.throttle = -1
            car_controls.brake = 1


        if spd > 170 and abs(angles[-1]) > 10:
            car_controls.throttle = -1
            car_controls.brake = 1




        angles = [0,] + [angle * plag for angle in sensing_info.track_forward_angles]

        if obs and obs[0][3] < 100:

            first_a, first_b, first_dist, first_m = obs[0]

            absolute_dist = (first_a**2 + first_b**2)**1/2
            start_road = int(- first_m - half_load_width)
            possible_path = [round(i * 0.1, 1) for i in range(start_road*10, int((start_road + half_load_width*2)*10))]
            
            new_path = [[-math.atan(angles[int(first_dist/10)]*math.pi / 180) * (x - first_b) + first_a, x] for x in possible_path]
            # print(new_path)
            same_position_obs = [[a,b,d,m] for a,b,d,m in obs if abs(first_dist-d) < 3]
            
            new_path2 = []
            for xx,yy in new_path:
                for a, b, d, m in same_position_obs:
                    pedding = 3.5 if angles[int(first_dist/10)] > 4 else 2.5
                    if not ((xx-a)**2 + (yy-b)**2)**1/2 > pedding:
                        break
                else:
                    new_path2.append([xx,yy])
            
            new_path3 = []
            for xx,yy in new_path2:
                flag = 0
                for a, b, d, m in same_position_obs:
                    if ((xx-a)**2 + (yy-b)**2)**1/2 < 3:
                        if yy < b:
                            flag = 1 # 목표점이 장애물의 왼쪽
                        else:
                            flag = 2 # 목표점이 장애물의 오른쪽
                        break
                
                if flag == 1:
                    if yy <= 0:
                        new_path3.append([xx-2,yy])
                    else:
                        new_path3.append([xx+2,yy])
                elif flag == 2:
                    if yy <= 0:
                        new_path3.append([xx+2,yy])
                    else:
                        new_path3.append([xx-2,yy])
                else:
                    new_path3.append([xx,yy])
            
            new_path3 = sorted(new_path3, key= lambda x : abs(x[1] - 0))
            # print(new_path3)
            for i in new_path3:
                if i[0] != 0:
                    if i[1] == 0:
                        final_thetas = 0
                    else:
                        final_thetas = math.atan((i[1] / i[0]) * 180 / math.pi - sensing_info.moving_angle)
                    break

            
            # print(final_thetas)
            car_controls.steering += final_thetas / 8






        # 충돌시 탈출 코드(수정 필요)

        if spd > 10:
            self.accident_step = 0
            self.recovery_count = 0
            self.accident_count = 0


        if sensing_info.lap_progress > 0.5 and self.accident_step == 0 and abs(spd) < 1.0:
            self.accident_count += 1
        
        if self.accident_count > 8:
            self.accident_step = 1

        if self.accident_step == 1:
            self.recovery_count += 1
            car_controls.steering = 0
            car_controls.throttle = -1
            car_controls.brake = 0

        if self.recovery_count > 20:
            self.accident_step = 2
            self.recovery_count = 0
            self.accident_count = 0

        if self.accident_step == 2:
            car_controls.steering = 0
            car_controls.throttle = 1
            car_controls.brake = 1
            if sensing_info.speed > -1:
                self.accident_step = 0
                car_controls.throttle = 1
                car_controls.brake = 0


        # 역방향 진행시 탈출 코드
        if not sensing_info.moving_forward and not (self.accident_count + self.accident_step + self.recovery_count) and spd > 0:
            self.uturn_count += 1
            if middle >= 0:
                self.uturn_step = 1
            else:
                self.uturn_step = -1
        
        if sensing_info.moving_forward:
            self.uturn_count = 0

        if self.uturn_count > 5:
            car_controls.steering = self.uturn_step
            car_controls.throttle = 0.5
        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        # print(car_controls.steering)
        return car_controls

    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name


if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)
