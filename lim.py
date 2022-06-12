import math
from DrivingInterface.drive_controller import DrivingController

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False

        # api or keyboard
        self.enable_api_control = False # True(Controlled by code) /False(Controlled by keyboard)
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
        x = int(self.half_road_limit * 2) * 4


        # 도로, 상대차, 장애물, 내차 모든 정보를 2차원 평면상에 표시
        MAP = [['-'] * x for _ in range(400)]

        # 차 중심 좌표 (car_a, car_b)
        car_a = 200
        car_b = x//2 + int(change(sensing_info.to_middle)//0.5)

        for i in range(10):
            for j in range(6):
                MAP[car_a - 5 +i][car_b - 3 +j] = 'A'
        
        # 장애물 중심 위치 (obstacle_a, obstacle_b)
        # print(sensing_info.track_forward_obstacles)
        if sensing_info.track_forward_obstacles:
            for i in sensing_info.track_forward_obstacles:
                if i['dist'] < 98:
                    obstacle_a = 200 - int(change(i['dist']) // 0.5)
                    obstacle_b = x//2 + int(change(i['to_middle']) // 0.5)
                    
                    for ii in range(2):
                        for jj in range(2):
                            MAP[obstacle_a - 1 + ii][obstacle_b - 1 + jj] = 'X'

        # 상대 차 중심 위치 (opp_a, opp_b)
        if sensing_info.opponent_cars_info and abs(sensing_info.opponent_cars_info['dist']) < 95:
            opp_a = 200 - int(change(sensing_info.opponent_cars_info['dist'] // 0.5))
            opp_b = x//2 + int(change(sensing_info.opponent_cars_info['to_middle']) // 0.5)

            for i in range(10):
                for j in range(6):
                    MAP[opp_a - 5 +i][opp_b - 3 +j] = 'B'



        
        
        print('pppppppppppppppppppppppppppppppppppppp')
        for i in MAP:
            print(''.join(i))





        middle = sensing_info.to_middle
        points = sensing_info.distance_to_way_points
        ts = [math.acos(abs(middle)/points[0])]
        for i in range(7):
            target = (points[i] ** 2 + points[i+1] ** 2 - 100) / (2 * points[i] * points[i+1])
            
            if abs(target) <= -1:
                target = -1
            elif abs(target) >= 1:
                target = 1
            ts.append(math.acos(target))
        
        value = sum(ts) * 180 / math.pi
        if middle > 0:
            if value > 90:
                theta = value - 90 - sensing_info.moving_angle
                car_controls.steering = theta / 90
        
        elif middle <= 0:
            theta = 90 - value - sensing_info.moving_angle
            car_controls.steering = theta / 45
        # print(middle, end="")
        # print('|||||', end="")
        # print(car_controls.steering)
        
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
