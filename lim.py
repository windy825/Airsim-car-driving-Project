
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

    def control_driving(self, car_controls, sensing_info):

        
        # def change(value):
        #     answer = round(abs(value), 1)
        #     if answer < int(answer) + 0.5:
        #         answer = int(answer)
        #     else:
        #         answer = int(answer) + 0.5
        #     return - answer if value < 0 else answer

        # 0.25 단위로 바꾸기
        def change(value):
            temp = int(abs(value) // 0.25)
            return - temp if value < 0 else temp

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
                if temp > 1:
                    temp = 1
                elif temp < -1:
                    temp = -1
                A =  math.asin(temp) * 180 / math.pi
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
            tg = 2
        elif spd < 130:
            tg = 2
        elif spd < 160:
            tg = 2
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
            if d < 0:
                n, k = -1, obj['dist']
                ang = (90 - angles[n+1]) * math.pi / 180
                obs.append([k * math.sin(ang) - m * math.cos(ang), -middle + k * math.cos(ang) + m * math.sin(ang)])
    
            else:
                n, k = int(d // 10), d % 10
                if n+2 > 20:
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
        if abs(angles[-1]) > 80 and spd > 80:
            car_controls.throttle = 0
            # if abs(theta) > 40:
            #     car_controls.steering = 1 if theta >= 0 else -1

##########################################################################################


        # 도로, 상대차, 장애물, 내차 모든 정보를 2차원 평면상에 표시
        # x = int(self.half_road_limit * 2) *7
        x = change(self.half_road_limit)*2 * 4
        MAP = [['-'] * x for _ in range(800)]

        # 차 중심 좌표 (car_a, car_b)
        car_a = 750
        car_b = x//2 + change(middle)

        # 도로가 시작되는 열 인덱스 리스트
        road_start_idx = [0] * 801

        # 중앙선 
        temp = [car_a, car_b - change(middle)]
        half_road = change(self.half_road_limit)
        for a, b in ways:
            xxx = car_a - change(a)
            yyy = car_b + change(b)
            if temp:
                rel_height = temp[0] - xxx
                rel_width = abs(yyy - temp[1])
                temp_value = 0 if rel_width == 0 else rel_height / rel_width
                cnt, giving = 0, 0
                if temp_value != 0:
                    moving = 1 if yyy - temp[1] < 0 else -1
                else:
                    moving = 0
                for j in range(temp[0] - xxx):
                    if cnt > temp_value:
                        giving += moving
                        cnt = 0

                    for v in range(-half_road, half_road+1, half_road):
                        if 0 <= yyy + giving + v < x:
                            MAP[xxx + j][yyy + giving + v] =  '|'
                            if 0 <= xxx+j < car_a and not road_start_idx[xxx+j]:
                                road_start_idx[xxx+j] = yyy + giving + v
                    cnt += 1
            # if 0 <= xxx and 0 <= yyy <x:
            #     MAP[xxx][yyy] = 'T'
            temp = [xxx, yyy]


        # 장애물 : 곡선정보 반영, 원으로 입력 (defalut r = 1m (4칸) + pedding r = 0)
        sensing_ob = []
        if obs:
            for i in range(len(obs)):
                ob_a = car_a - change(obs[i][0])
                ob_b = car_b + change(obs[i][1])
                # MAP[ob_a][ob_b] = 'X'
                for o in range(5):
                    if 0 <= ob_b + o - 2 < x:
                        MAP[ob_a - 4][ob_b + o - 2] = 'X'
                        MAP[ob_a + 4][ob_b + o - 2] = 'X'
                for o in range(7):
                    if 0 <= ob_b + o - 3 < x:
                        MAP[ob_a - 3][ob_b + o - 3] = 'X'
                        MAP[ob_a + 3][ob_b + o - 3] = 'X'
                for oi in range(ob_a - 2, ob_a + 3):
                    for o in range(9):
                        if 0 <= ob_b + o - 4 < x:
                            MAP[oi][ob_b + o - 4] = 'X'
                            MAP[oi][ob_b + o - 4] = 'X'

                sensing_ob.append([ob_a, ob_b, sensing_info.track_forward_obstacles[i]['dist']])

        # 내 차 찍기
        if 0 <= car_b < x:
            MAP[car_a][car_b] = 'A' 

        # 장애물 극복 라인 생성

        if sensing_ob and sensing_ob[0][2] < 150:
            first_ob_a, first_ob_b, first_dist = sensing_ob[0]
            temp = [(i, road_start_idx[i]) for i in range(first_ob_a - 40, first_ob_a + 40) if i < 800]
            start_a, start_b = min(temp, key = lambda x : (x[0] - first_ob_a)**2 + (x[1] - first_ob_b)**2)

            MAP[start_a][start_b] = 'T'
            temp = first_ob_a, first_ob_b
            xxx, yyy = start_a, start_b

            rel_height = temp[0] - xxx
            rel_width = abs(yyy - temp[1])
            temp_value = 0 if rel_height == 0 else rel_width / abs(rel_height)
            cnt, giving = 0, 0

            if temp_value != 0:
                moving = 1 if yyy - temp[1] < 0 else -1
            else:
                moving = 0

            for j in range(half_road*2):
                if cnt > temp_value:
                    giving += moving
                    cnt = 0
                
                if 0<= yyy + giving< x:
                    if rel_height < 0 and xxx + giving < car_a and 0<= yyy - j and MAP[xxx + giving][yyy - j] != 'X':                    
                        MAP[xxx + giving][yyy - j] =  'O'
                    elif xxx+giving < car_a and yyy + j < x and MAP[xxx + giving][yyy + j] != 'X':                    
                        MAP[xxx + giving][yyy + j] =  'O'
                cnt += 1


 


        # # 장애물 판단 라인 따는 곳 설정
        # if i_idx:
        #     # my_car_b = car_b + (road_start_idx[i_idx] - road_start_idx[car_a - 1])
        #     # my_car_b = car_b
        #     # cnt = 0
        #     # for car_position in range(my_car_b-1, my_car_b+2):
        #     #     if 0 <= car_position < x and path_arr[car_position]:
        #     #         cnt += 1
        #     # if cnt == 3:
        #     #     i_idx -= 4
        #     if path_arr[car_b]:
        #         i_idx -= 4
        
        
        # # 가까운 2번째 장애물
        # if i_idx and i_idx-5 > 0:
        #     i_cnt = 0
        #     for i2 in range(i_idx-7, i_idx-47 , -1):
        #         if i_cnt == 5:
        #             break

        #         if 'X' in MAP[i2]:
        #             i_cnt += 1

        #             # start2 = road_start_idx[i2] -5
        #             # end2 = start2 + 2 * half_road +10
        #             for j2 in range(x):
        #                 if 0 <= j2 - (road_start_idx[i2] - road_start_idx[i_idx]) < x and 0 <= j2 < x and MAP[i2][j2] in '/X':
        #                     path_arr[j2 - (road_start_idx[i2] - road_start_idx[i_idx])] = 0

    
        # if flag:
        #     # if spd > 100 and car_a - i_idx < 40:
        #     #     car_controls.throttle = 0.1



        #     # 장애물 판단 라인의 범위 설정 설정
        #     # start = road_start_idx[i_idx]
        #     # end = start + 2 * half_road +2
        #     # for kk1 in range(len(path_arr)):
        #     #     if kk1 < start or kk1 > end:
        #     #         path_arr[kk1] = 0

        #     # 맵에 경로 표시
        #     for kk2 in range(len(path_arr)):
        #         if path_arr[kk2]:
        #             MAP[i_idx][kk2] = 'O'


        #     # 가능한 경로를 도착지로 한 핸들링 값 + 경로 선택
        #     new_arr = []
        #     for b in range(len(path_arr)):
        #         if path_arr[b]:
        #             new_arr.append(math.atan((b - car_b) / (car_a - i_idx)) * 180 / math.pi - sensing_info.moving_angle)


        #     path_arr = [(math.atan((b - car_b) / (car_a - i_idx)) * 180 / math.pi - sensing_info.moving_angle) for b in range(len(path_arr)) if path_arr[b]]
        #     target = sorted([tt/60 for tt in path_arr], key= lambda x : abs(x - car_controls.steering))
        #     # car_controls.steering = target[0]
        #     for tt in target:
        #         if tt < 0.4:
        #             car_controls.steering = tt
        #             break
        #     else:
        #         car_controls.steering = target[-1]


        for i in range(100, car_a+1):
                # if 'O' in MAP[i]:
                #     print(i, end='')
            print(''.join(MAP[i][car_b-100 : car_b+100]))
        print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
        


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
