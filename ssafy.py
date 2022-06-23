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

            

        # 조절해서 쓰기
        if spd < 30 and sensing_info.lap_progress > 1:
            tg = 0
        elif spd < 140:
            tg = 3
        elif spd < 170:
            tg = 5
        else:
            tg = 7

        theta = math.atan(ways[tg][1] / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle

        if abs(theta) < 50 or sensing_info.lap_progress >= 99.5:
            if spd < 140:
                car_controls.steering = theta / 80
            else:
                car_controls.steering = theta / 110

        else:
            r = max(abs(ways[tg-1][0]), abs(ways[tg-1][1]))
            alpha = math.asin(math.sqrt(ways[tg-1][0] ** 2 + ways[tg-1][1] ** 2) / (2 * r)) * 2
            beta = alpha * spd * 0.12 / r
            beta = beta if theta >= 0 else -beta
            car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) * 1.1


        # 끝
        # 좌표는 ways에 순서대로

        if abs(angles[int(spd//25)]) > 42 and spd > 100:
            car_controls.throttle = -0.5 if spd < 120 else -1
            car_controls.brake = 1

        if spd > 170 and abs(angles[-1]) > 10:
            car_controls.throttle = -1
            car_controls.brake = 1


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
