import math

from numpy import False_
from DrivingInterface.drive_controller import DrivingController


before = 0
accident_step = 0
recovery_count = 0
accident_count = 0
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

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):
        global accident_count, recovery_count, accident_step

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

        # half_load_width = self.half_road_limit - 1.25
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
                obs.append([k * math.sin(ang) - m * math.cos(ang), -middle + k * math.cos(ang) + m * math.sin(ang)])
    
            else:
                n, k = int(d // 10), d % 10
                if n+2 > 10:
                    break
                ang = (90 - angles[n+1] * plag) * math.pi / 180
                obs.append([ways[n][0] + k * math.sin(ang) - m * math.cos(ang), ways[n][1] + k * math.cos(ang) + m * math.sin(ang)])

            

        # 주행 코드

        if spd < 30 and sensing_info.lap_progress > 1:
            tg = 0
        elif spd < 140:
            tg = 3
        elif spd < 170:
            tg = 5
        else:
            tg = 7



        if abs(angles[tg]) < 10:
            ## 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
            for i in range(20):
                if abs(angles[i]) > 60:
                    break
            
            ## 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
            if 12 < i < 19:
                if angles[i+1] * plag < 0:
                    middle_add = (-3.5 + middle ) / 80 * -1
                else:
                    middle_add = (3.5 + middle) / 80 * -1
            elif abs(middle) > 5:
                middle_add = -middle / 100
            else:
                middle_add = 0

            ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
            set_steering = (angles[tg] * plag - sensing_info.moving_angle) / (spd * 0.65 + 0.001) + middle_add

            car_controls.steering = set_steering
        else:
            theta = math.atan(ways[tg][1] / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle
            car_controls.steering = theta / 80 if spd < 140 else theta / 100

        if abs(angles[int(spd//20)]) > 40 and spd > 100:
            car_controls.throttle = -1
            car_controls.brake = 1

        if spd > 170 and abs(angles[-1]) > 10:
            car_controls.throttle = -1
            car_controls.brake = 1
        
        if sensing_info.lap_progress > 99.4:
            car_controls.steering = -sensing_info.moving_angle / 80

        # 충돌시 탈출 코드(수정 필요)
        if spd > 10:
            accident_step = 0
            recovery_count = 0
            accident_count = 0

        if sensing_info.lap_progress > 0.5 and accident_step == 0 and abs(spd) < 1.0:
            accident_count += 1
        
        if accident_count > 8:
            accident_step = 1

        if accident_step == 1:
            recovery_count += 1
            car_controls.steering = 0
            car_controls.throttle = -1
            car_controls.brake = 0

        if recovery_count > 20:
            accident_step = 2
            recovery_count = 0
            accident_count = 0

        if accident_step == 2:
            car_controls.steering = 0
            car_controls.throttle = 1
            car_controls.brake = 1
            if sensing_info.speed > -1:
                accident_step = 0
                car_controls.throttle = 1
                car_controls.brake = 0


        # if not sensing_info.moving_forward:


        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
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
