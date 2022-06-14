import math
from DrivingInterface.drive_controller import DrivingController

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================
        # ================================== #
        # Editing area starts from here
        self.is_debug = False
        # api or keyboard
        self.enable_api_control = False # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)
        self.track_type = 99
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        # Editing area ends
        # ==========================================================#
        super().__init__()




######################################################################################################################




    def control_driving(self, car_controls, sensing_info):
        # 0.5 단위로 바꾸기
        def change(value):
            answer = round(abs(value), 1)
            if answer < int(answer) + 0.5:
                answer = int(answer)
            else:
                answer = int(answer) + 0.5
            return - answer if value < 0 else answer

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
        if middle >= 0:
            points = [middle,] + sensing_info.distance_to_way_points
            angles = [0,] + sensing_info.track_forward_angles
            bo = [90, ]
            ts = []
            for i in range(10):
                C = 180 - bo[i] - (angles[i+1] - angles[i])
                A =  math.asin((points[i] * math.sin(C * math.pi / 180)) / points[i+1]) * 180 / math.pi
                bo.append(A)
                target = 180 - C - A
                ts.append(target)

            # way point 각도
            ways = []
            for j in range(10):
                ways.append([points[j+1] * math.sin(sum(ts[:j+1]) * math.pi / 180), - points[j+1] * math.cos(sum(ts[:j+1]) * math.pi / 180)])



            theta = sum(ts[:6 if sensing_info.speed < 120 else 7]) - 90 - sensing_info.moving_angle
            car_controls.steering = theta / (120 if sensing_info.speed < 120 else 80) 
            
        else:
            points = [-middle,] + sensing_info.distance_to_way_points
            angles = [0, ] + [-angle for angle in sensing_info.track_forward_angles]
            bo = [90, ]
            ts = []
            for i in range(10):
                C = 180 - bo[i] - (angles[i+1] - angles[i])
                A = math.asin((points[i] * math.sin(C * math.pi / 180)) / points[i+1]) * 180 / math.pi
                bo.append(A)
                target = 180 - C - A
                ts.append(target)

            ways = []
            for j in range(10):
                ways.append([points[j+1] * math.sin(sum(ts[:j+1]) * math.pi / 180), points[j+1] * math.cos(sum(ts[:j+1]) * math.pi / 180)])
            
            theta = 90 - sum(ts[:6 if sensing_info.speed < 120 else 7]) - sensing_info.moving_angle
            car_controls.steering = theta / (120 if sensing_info.speed < 100 else 75) 

        # 끝
        # 좌표는 ways에 순서대로


        # for j in range(10):
        #     print(sum(ts[:j+1]), ways[j])
        # print('---------------')


        # 장애물 좌표 시작 (무조건 ways 계산 다음에 할 것)
        obs = []
        near = points[0] * math.sin(ts[0] * math.pi / 180) / math.sin(bo[1] * math.pi / 180) if bo[1] > 0 else 0
        # print(points[0], math.sin(ts[0] * math.pi / 180), math.sin(bo[1] * math.pi / 180))
        for obj in sensing_info.track_forward_obstacles:
            d, m = obj['dist'] - near, obj['to_middle']
            if d <= 0:
                n, k = -1, obj['dist']
            else:
                n, k = int(d // 10), d % 10
            if n+2 > 10:
                break
            ang = angles[n+1]
            obs.append([ways[n+1][0] + k * math.cos(ang) - m * math.sin(ang), ways[n+1][1] + k * math.sin(ang) - m * math.cos(ang)])

        
        # print(obs)


########################### MAP 생성 #####################################################################################################

        # 도로, 상대차, 장애물, 내차 모든 정보를 2차원 평면상에 표시
        x = int(self.half_road_limit * 2) * 6
        MAP = [['-'] * x for _ in range(400)]


        # 차 중심 좌표 (car_a, car_b)
        car_a = 200
        car_b = x//2 + int(change(sensing_info.to_middle)//0.5)

        
        # 장애물
        if obs:
            for a, b in obs:
                newa = car_a - int(change(a) // 0.5)
                newb = car_b - int(change(b) // 0.5)
                if 0 <= newa and 0 <= newb < x:
                    MAP[newa][newb] = 'X'

        
        # 중앙선
        temp = []
        for a, b in ways:
            half_road = int(change(self.half_road_limit) // 0.5)
            xxx = car_a - int(change(a) // 0.5)
            yyy = car_b + int(change(b) // 0.5)
            
            if temp:
                temp_value = 0 if yyy - temp[1] == 0 else int((temp[0] - xxx) / abs(yyy - temp[1]) + 0.5)
                cnt, giving = 0, 0
                if temp_value != 0:
                    moving = 1 if yyy - temp[1] < 0 else -1
                else:
                    moving = 0

                for j in range(temp[0] - xxx):
                    if cnt == temp_value:
                        giving += moving
                        cnt = 0
                    for v in range(-half_road, half_road+1, half_road):
                        
                        if 0 <= yyy + giving + v < x:
                            MAP[xxx + j][yyy + giving + v] =  '|'
                    cnt += 1
            temp = [xxx, yyy]


        # 내 차 찍기
        for i in range(8):
            for j in range(4):
                MAP[car_a - 4 +i][car_b - 2 +j] = 'A'


        # 상대 차 중심 위치 (opp_a, opp_b)
        if sensing_info.opponent_cars_info and abs(sensing_info.opponent_cars_info['dist']) < 95:
            opp_a = 200 - int(change(sensing_info.opponent_cars_info['dist'] // 0.5))
            opp_b = x//2 + int(change(sensing_info.opponent_cars_info['to_middle']) // 0.5)

            for i in range(10):
                for j in range(6):
                    MAP[opp_a - 5 +i][opp_b - 3 +j] = 'B'




        print('ㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁㅁ')
        for i in range(50,210):
            print(''.join(MAP[i]))




        
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
