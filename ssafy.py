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

        half_load_width = self.half_road_limit - 3
        car_controls.throttle = 1
        car_controls.brake = 0


        # way points 좌표 시작
        middle = sensing_info.to_middle
        spd = sensing_info.speed
        angles = sensing_info.track_forward_angles



        ################################## 삽입 부분 #########################################

        # 인식거리 설정
        ob_start = 0
        ob_end = 150

        # 범위 
        ob_line = [round(i * 0.1, 1) for i in range(-int(half_load_width)*10, int(half_load_width)*10)]
        ob_line2 = []
        cnt = 0
        # 인식거리 안의 장애물 등록
        for obj in sensing_info.track_forward_obstacles:
            ob_dist, ob_middle = obj['dist'], obj['to_middle']

            if ob_start <= ob_dist <= ob_end:
                if abs(angles[int(ob_dist/10)]) > 7 and ob_dist > 10:
                    ped = 3
                else:
                    ped = 2.25
                ob_line = [i for i in ob_line if not ob_middle-ped <= i <= ob_middle+ped]
                cnt += 1
            
            if cnt == 2:
                break

            if not ob_line:
                ob_line = ob_line2[:]
                break
            else:
                ob_line2 = ob_line[:]
        


        target = min(ob_line,key = lambda x : abs(x - middle))
        p = - (middle - target) * 0.1
        i = p ** 2 * 0.05 if p >= 0 else - p ** 2 * 0.05 

        middle_add = 0.5 * p + 0.3 * i


        ################################## 삽입 부분 #########################################
        

        # 주행 코드

        if spd < 50 and sensing_info.lap_progress > 1:
            tg = 0
        elif spd < 120:
            tg = 1
        elif spd < 180:
            tg = 2
        else :
            tg = 4

        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        if (angles[tg]) < 45:
            if spd < 80:
                set_steering = (angles[tg] - sensing_info.moving_angle) / 80
            else:
                set_steering = (angles[tg] - sensing_info.moving_angle) / 80
            car_controls.steering = set_steering
            car_controls.steering += middle_add
        else:
            if angles[tg] < 0:
                r = self.half_road_limit - 1.25 + middle - 1
                beta = - math.pi * spd * 0.1 / r
                car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180)
            else:    
                r = self.half_road_limit - 1.25 - middle - 1
                beta = math.pi * spd * 0.1 / r
                car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180)



        if abs(angles[int(spd//20)]) > 40 and spd > 85:
            car_controls.throttle = 0 if spd < 120 else -1
            car_controls.brake = 1


        if spd > 170 and abs(angles[-1]) > 10:
            car_controls.throttle = -1
            car_controls.brake = 1


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


        # 주행 코드

        if spd < 20 and sensing_info.lap_progress > 3:
            tg = 0
        elif spd < 140:
            tg = 3
        elif spd < 170:
            tg = 5
        else:
            tg = 7


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
