import math, time

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

        half_road_width = self.half_road_limit - 1.25
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
            elif abs(middle) > half_road_width:
                middle_add = -middle / 100
            else:
                middle_add = 0

            ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
            # set_steering = (angles[tg] * plag - sensing_info.moving_angle) / (spd * 0.65 + 0.001) + middle_add
            set_steering = (angles[tg] * plag - sensing_info.moving_angle) / (spd * 0.65 + 0.001)

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


        ################################## 삽입 부분 #########################################
        def change(value):
            temp = int(abs(value) // 0.25)
            return - temp if value < 0 else temp

        # 도로, 상대차, 장애물, 내차 모든 정보를 2차원 평면상에 표시
        # x = int(self.half_road_limit * 2) *7
        x = change(self.half_road_limit)*6
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
                        if 0<xxx + j<car_a and 0 <= yyy + giving + v < x:
                            MAP[xxx + j][yyy + giving + v] =  '|'
                            if 0 <= xxx+j < car_a and not road_start_idx[xxx+j]:
                                road_start_idx[xxx+j] = yyy + giving + v
                    cnt += 1
            # if 0 <= xxx and 0 <= yyy <x:
            #     MAP[xxx][yyy] = 'T'
            temp = [xxx, yyy]


        # 장애물 : 곡선정보 반영, 원으로 입력 (defalut r = 1m (4칸) + pedding r = 0)


        sensing_ob = []
        path_arr = []
        if obs:
            for i in range(len(obs)):
                ob_a = car_a - change(obs[i][0])
                ob_b = car_b + change(obs[i][1])

                for o in range(0,9):
                    if 0 <= ob_b + o - 4 < x:
                        MAP[ob_a - 9][ob_b + o - 4] = 'X'
                        MAP[ob_a + 9][ob_b + o - 4] = 'X'
                        path_arr.append([ob_a-9,ob_b+o-4])
                        path_arr.append([ob_a+9,ob_b+o-4])

                for o in range(0,11):
                    if 0 <= ob_b + o - 5 < x:
                        MAP[ob_a - 8][ob_b + o - 5] = 'X'
                        MAP[ob_a + 8][ob_b + o - 5] = 'X'
                        path_arr.append([ob_a-8,ob_b+o-5])
                        path_arr.append([ob_a+8,ob_b+o-5])


                for o in range(0,13):
                    if 0 <= ob_b + o - 6 < x:
                        MAP[ob_a - 7][ob_b + o - 6] = 'X'
                        MAP[ob_a + 7][ob_b + o - 6] = 'X'
                        path_arr.append([ob_a-7,ob_b+o-6])
                        path_arr.append([ob_a+7,ob_b+o-6])

                for o in range(0,15):
                    if 0 <= ob_b + o - 7 < x:
                        MAP[ob_a - 6][ob_b + o - 7] = 'X'
                        MAP[ob_a + 6][ob_b + o - 7] = 'X'
                        path_arr.append([ob_a-6,ob_b+o-7])
                        path_arr.append([ob_a+6,ob_b+o-7])

                for oi in range(2):
                    for o in range(0,15):
                        if 0 <= ob_b + o - 7 < x:
                            MAP[ob_a + oi - 5][ob_b + o - 7] = 'X'
                            MAP[ob_a + oi + 4][ob_b + o - 7] = 'X'
                            path_arr.append([ob_a + oi - 5,ob_b+o-7])
                            path_arr.append([ob_a + oi + 4,ob_b+o-7])

                for oi in range(7):
                    for o in range(0,15):
                        if 0 <= ob_b + o - 7 < x:
                            MAP[ob_a + oi + - 3][ob_b + o - 7] = 'X'
                            path_arr.append([ob_a + oi + - 3,ob_b+o-7])



                if 100 <= ob_a < car_a and 0 <= ob_b < x:
                    sensing_ob.append([ob_a, ob_b, sensing_info.track_forward_obstacles[i]['dist']])

        # 내 차 찍기
        for i in range(10):
            if 0 <= car_b - 5 + i < x:
                MAP[car_a][car_b - 5 + i] = 'A' 

        #         if road_start_idx[car_a]+ 5 < car_b - 25 and car_b + 25 < road_start_idx[car_a] + half_road*2 - 5:
        #             if car_b - 25 <= b <= car_b + 25:

        #         elif road_start_idx[a]+ 5 >= car_b - 25:
        #             if road_start_idx[a] + 5 <= b <= road_start_idx[a] + 55:
                
        #         elif car_b + 25 >= road_start_idx[a] + half_road*2 - 5:
        #             if road_start_idx[a] + half_road * 2 - 55 <= b <= road_start_idx[a] + half_road * 2 - 5:

        # path_line = [-1] * (x+1)
        # for bb in range(len(path_line)):
        #     if car_b - 30 
        #     if car_b - 30 <= bb <= car_b + 30:
        #         path_line[bb] = sum(1 for aa in range(800) if MAP[aa][bb] == 'X')




        if sensing_ob and 4 < sensing_ob[0][2] < 60:

            path_arr = []
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


            # if temp_value != 0:
            #     moving = 1 if yyy - temp[1] < 0 else -1
            # else:
            moving = 0


            xxx, yyy = first_ob_a, first_ob_b
            for j in range(150):
                if cnt > temp_value:
                    giving += moving
                    cnt = 0
                if 0 <= yyy + giving < x:
                    if rel_height < 0 and 0 <= xxx + giving < car_a and 0 <= yyy - j and MAP[xxx + giving][yyy - j] !=  'X':                    
                        path_arr.append([xxx + giving, yyy - j])
                    elif 0 <= xxx + giving < car_a and yyy + j < x and MAP[xxx + giving][yyy + j] !=  'X':                    
                        path_arr.append([xxx + giving, yyy + j])

                    if rel_height < 0 and 0 <= xxx - giving < car_a and 0 <= yyy + j < x and MAP[xxx - giving][yyy + j] !=  'X':                    
                        path_arr.append([xxx - giving, yyy + j])
                    elif 0 <= xxx - giving < car_a and 0 <= yyy - j < x and MAP[xxx - giving][yyy - j] !=  'X':                    
                        path_arr.append([xxx - giving, yyy - j])
                cnt += 1



            # 위치 전방 구간만 확인하기
            path_arr2 = []
            for a,b in path_arr:

                if road_start_idx[a]+ 8 < car_b - 25 and car_b + 25 < road_start_idx[a] + half_road*2 - 8:
                    if car_b - 25 <= b <= car_b + 25:
                        path_arr2.append([a,b])

                elif road_start_idx[a]+ 8 >= car_b - 25:
                    if road_start_idx[a] + 8 <= b <= road_start_idx[a] + 58:
                        path_arr2.append([a,b])
                
                elif car_b + 25 >= road_start_idx[a] + half_road*2 - 8:
                    if road_start_idx[a] + half_road * 2 - 58 <= b <= road_start_idx[a] + half_road * 2 - 8:
                        path_arr2.append([a,b])



            # if car_b <= first_ob_b:
            #     path_arr2 = [[a,b] for a, b in path_arr2 if b <= first_ob_b]
            # else:
            #     path_arr2 = [[a,b] for a, b in path_arr2 if b > first_ob_b]

            path_arr3 = path_arr2[:]
            # for a,b in path_arr2:
            #     if abs(car_b - b) < 5:
            #         path_arr3.append([a-10, b])
            #     else:
            #         path_arr3.append([a, b])

            for a,b in path_arr3:
                MAP[a][b] = 'O'

            

            if path_arr3:
                # addings = road_start_idx[first_ob_a] - road_start_idx[start_a] if road_start_idx[first_ob_a] - road_start_idx[start_a] > 10 else 0
                addings = 0
                path_arr3 = [(math.atan((b - car_b + addings) / (car_a - a)) * 180 / math.pi - sensing_info.moving_angle) for a, b in path_arr3 if car_a != a]
                
                car_controls.steering = min(path_arr3, key= lambda x : abs(x/(spd*0.4 if spd != 0 else 54) - car_controls.steering)) / (spd*0.4 if spd != 0 else 54)
                
                

                for i in range(600,car_a+2):
                    print(''.join(MAP[i]))

                if spd > 100:
                    car_controls.brake = 0.1

                # if spd > 130 and  car_controls.steering < 0.05:
                #     car_controls.steering *= 1.3
                #     car_controls.throttle = 0.2

        if spd > 65 and abs(car_controls.steering) > 0.35:
            car_controls.throttle = 0
            car_controls.brake = 0.7

        if len(sensing_ob) > 1:
            if spd > 70 and 4 < abs(sensing_ob[1][2] - sensing_ob[0][2]) < 25:
                print('YESSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS')
                car_controls.throttle = 0.1

        if len(sensing_ob) > 2:
            if spd > 70 and 4 < abs(sensing_ob[2][2] - sensing_ob[1][2]) < 25:
                print('YESSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS')
                car_controls.throttle = 0.1


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
