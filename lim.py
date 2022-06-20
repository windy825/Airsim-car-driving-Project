
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
        self.enable_api_control = True # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)
        self.track_type = 99
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        # Editing area ends
        # ==========================================================#
        super().__init__()

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
        spd = sensing_info.speed


        if middle >= 0:
            points = [middle,] + sensing_info.distance_to_way_points
            angles = [0,] + sensing_info.track_forward_angles
            bo = [90, ]
            ts = []
            for i in range(20):
                C = 180 - bo[i] - (angles[i+1] - angles[i])
                temp = points[i] * math.sin(C * math.pi / 180) / points[i+1]
                A =  math.asin(temp if abs(temp) <= 1 else int(temp)) * 180 / math.pi
                bo.append(A)
                target = 180 - C - A
                ts.append(target)

            # way point 각도
            ways = []
            for j in range(20): 
                ways.append([points[j+1] * math.sin(sum(ts[:j+1]) * math.pi / 180), - points[j+1] * math.cos(sum(ts[:j+1]) * math.pi / 180)])

        else:
            points = [-middle,] + sensing_info.distance_to_way_points
            angles = [0, ] + [-angle for angle in sensing_info.track_forward_angles]
            bo = [90, ]
            ts = []
            for i in range(20):
                C = 180 - bo[i] - (angles[i+1] - angles[i])
                temp = points[i] * math.sin(C * math.pi / 180) / points[i+1]
                A =  math.asin(temp if abs(temp) <= 1 else int(temp)) * 180 / math.pi
                bo.append(A)
                target = 180 - C - A
                ts.append(target)

            ways = []
            for j in range(20):
                ways.append([points[j+1] * math.sin(sum(ts[:j+1]) * math.pi / 180), points[j+1] * math.cos(sum(ts[:j+1]) * math.pi / 180)])
            

        # 조절해서 쓰기
        if spd < 110:
            tg = 3
        elif spd < 130:
            tg = 5
        elif spd < 150:
            tg = 6
        else:
            tg = 7




        target = ways[tg][:]

        # 아래 부분에 원하는 좌표 넣기

        # target = 

        theta = math.atan(target[1] / target[0]) * 180 / math.pi - sensing_info.moving_angle

        # theta 값의 범위 적당히 조정
        # 마찬가지로 steering도 상수값 조정하면 됨
        if spd < 100:
            car_controls.steering = theta / 120
        else:
            car_controls.steering = theta / 100




        # 끝
        # 좌표는 ways에 순서대로

        obs = []
        near = points[0] * math.cos((90 - angles[1]) * math.pi / 180) + points[1] * math.cos(bo[1] * math.pi / 180)
        for obj in sensing_info.track_forward_obstacles:
            d, m = obj['dist'] - near, obj['to_middle']
            if d <= 0:
                n, k = -1, obj['dist']
                ang = (90 - angles[n+1]) * math.pi / 180
                obs.append([k * math.sin(ang) - m * math.cos(ang), -middle + k * math.cos(ang) + m * math.sin(ang)])
    
            else:
                n, k = int(d // 10), d % 10
                if n+2 > 10:
                    break
                ang = (90 - angles[n+1]) * math.pi / 180
                obs.append([ways[n][0] + k * math.sin(ang) - m * math.cos(ang), ways[n][1] + k * math.cos(ang) + m * math.sin(ang)])


        # 아웃 인 아웃 구현
        # if abs(sensing_info.moving_angle) < 5 and abs(theta) < 5:
        #     car_controls.steering = 0
        #     if angles[-1] > 90 and middle > -5:
        #         pass
        #     elif angles[-1] < -90 and middle < 5:


        if not abs(theta) < 10 and spd > 160:
            car_controls.throttle = 0.1
        if abs(angles[-1]) > 120 and spd > 100:
            car_controls.throttle = 0
            car_controls.brake = 1
            if abs(theta) > 60:
                car_controls.steering = 1 if theta >= 0 else -1




        # 도로, 상대차, 장애물, 내차 모든 정보를 2차원 평면상에 표시
        x = int(self.half_road_limit * 2) * 8
        MAP = [['-'] * x for _ in range(400)]

        # 차 중심 좌표 (car_a, car_b)
        car_a = 200
        car_b = x//2 + int(change(middle)//0.5)

        # 도로가 시작되는 인덱스 리스트
        road_start_idx = [0] * (car_a+1)
        # 중앙선
        temp = [car_a, car_b - int(change(middle) // 0.5)]
        half_road = int(change(self.half_road_limit) // 0.5)
        for a, b in ways:
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
                            if 0 <= xxx+j < car_a and not road_start_idx[xxx+j]:
                                road_start_idx[xxx+j] = yyy + giving + v
                    cnt += 1
            temp = [xxx, yyy]


        # 장애물
        if obs:
            for a, b in obs:
                newa = car_a - int(change(a) // 0.5)
                newb = car_b + int(change(b) // 0.5)
                for ii in range(4):
                    for jj in range(12):
                        if 0 <= newa - 2 + ii and 0 <= newb -6 + jj < x:
                            if MAP[newa - 2 + ii][newb -6 + jj] == '-':
                                MAP[newa - 2 + ii][newb -6 + jj] = 'X'
                            else:
                                MAP[newa - 2 + ii][newb -6 + jj] = '/'


        # 내 차 찍기
        # MAP[car_a][car_b] = 'A'
        MAP[car_a][car_b] = 'A' 


        # 가능한 통로 계산
        path_arr = [1] * (x)
        flag = 0
        i_cnt = 0
        i_idx = 0
 
        for i in range(car_a -4, 0, -1):
            if i_cnt == 3:
                break
            if 'X' in MAP[i]:
                flag = 1
                i_cnt += 1
                if not i_idx:
                    i_idx = i

                start = road_start_idx[i] -2
                end = start + 2 * half_road +4
                for j in range(start, end):
                    if 0 <= j < x and MAP[i][j] in '/X':
                        path_arr[j] = 0

        start = road_start_idx[i] -2
        end = start + 2 * half_road +4
        for kk1 in range(len(path_arr)):
            if kk1 <= start or kk1 > end:
                path_arr[kk1] = 0

        if not path_arr[car_b]:
            if i_idx -4 < 0:
                i_idx = 0
            else:
                i_idx -= 4
        
        for kk2 in range(len(path_arr)):
            if path_arr[kk2]:
                MAP[i_idx][kk2] = 'O'


        path_arr = [(math.atan((b - car_b) / (car_a - i_idx)) * 180 / math.pi - sensing_info.moving_angle) for b in range(len(path_arr)) if path_arr[b]]
        
        if flag:
            if spd > 135:
                car_controls.throttle = 0
                car_controls.brake = 0.5
            elif spd < 100:
                car_controls.brake = 0

            target = []
            if spd < 100:
                xxx = 65
            elif spd < 125:
                xxx = 60
            elif spd < 135:
                xxx = 50
            else:
                xxx = 40
            for a in path_arr:
                target.append(a / 60)
            car_controls.steering = min(target, key= lambda x : abs(x - car_controls.steering))


        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

    

        return car_controls


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
