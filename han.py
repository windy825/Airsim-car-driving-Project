from DrivingInterface.drive_controller import DrivingController
import math


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = True

        # api or keyboard
        self.enable_api_control = True  # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        self.accident_step = 0

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

        ###########################################################################

        # 기본설정
        # half_load_width = self.half_road_limit - 1.25
        car_controls.throttle = 1
        car_controls.brake = 0
        car_controls.steering = 0

        # 충돌 체크
        if sensing_info.speed > 10:
            self.accident_step = 0
            self.recovery_count = 0
            self.accident_count = 0

        if sensing_info.lap_progress > 0.5 and self.accident_step == 0 and abs(sensing_info.speed) < 1.0:
            self.accident_count += 1

        if self.accident_count > 8:
            self.accident_step = 1

        if self.accident_step == 0:
            # way points 좌표 시작
            middle = sensing_info.to_middle
            if middle >= 0:
                points = [middle, ] + sensing_info.distance_to_way_points
                angles = [0, ] + sensing_info.track_forward_angles
                bo = [90, ]
                ts = []
                for i in range(10):
                    C = 180 - bo[i] - (angles[i + 1] - angles[i])
                    A = math.asin((points[i] * math.sin(C * math.pi / 180)) / points[i + 1]) * 180 / math.pi
                    bo.append(A)
                    target = 180 - C - A
                    ts.append(target)

                # way point 각도
                ways = []
                for j in range(10):
                    ways.append([points[j + 1] * math.sin(sum(ts[:j + 1]) * math.pi / 180),
                                 - points[j + 1] * math.cos(sum(ts[:j + 1]) * math.pi / 180)])

                theta = sum(ts[:6 if sensing_info.speed < 120 else 7]) - 90 - sensing_info.moving_angle
                car_controls.steering = theta / (120 if sensing_info.speed < 120 else 80)

            else:
                points = [-middle, ] + sensing_info.distance_to_way_points
                angles = [0, ] + [-angle for angle in sensing_info.track_forward_angles]
                bo = [90, ]
                ts = []
                for i in range(10):
                    C = 180 - bo[i] - (angles[i + 1] - angles[i])
                    A = math.asin((points[i] * math.sin(C * math.pi / 180)) / points[i + 1]) * 180 / math.pi
                    bo.append(A)
                    target = 180 - C - A
                    ts.append(target)

                ways = []
                for j in range(10):
                    ways.append([points[j + 1] * math.sin(sum(ts[:j + 1]) * math.pi / 180),
                                 points[j + 1] * math.cos(sum(ts[:j + 1]) * math.pi / 180)])

                theta = 90 - sum(ts[:6 if sensing_info.speed < 120 else 7]) - sensing_info.moving_angle
                car_controls.steering = theta / (120 if sensing_info.speed < 100 else 75)

        elif self.accident_step == 1:
            car_controls.throttle = -1
            ob_to_middle1 = []
            ob_to_middle2 = []
            ob_to_middle3 = []
            for obstacle in sensing_info.track_forward_obstacles:
                if 0 < obstacle['dist'] < 11:
                    if obstacle['to_middle'] <= -(self.half_road_limit - 1.25) / 3 or \
                            obstacle['to_middle'] - 1 <= -(self.half_road_limit - 1.25) / 3:
                        ob_to_middle1.append(obstacle['to_middle'])
                    if -(self.half_road_limit - 1.25) / 3 < obstacle['to_middle'] <= (
                            self.half_road_limit - 1.25) / 3 or \
                            -(self.half_road_limit - 1.25) / 3 < obstacle['to_middle'] - 1 <= (
                            self.half_road_limit - 1.25) / 3 or \
                            -(self.half_road_limit - 1.25) / 3 < obstacle['to_middle'] + 1 <= (
                            self.half_road_limit - 1.25) / 3:
                        ob_to_middle2.append(obstacle['to_middle'])
                    if (self.half_road_limit - 1.25) / 3 < obstacle['to_middle'] or \
                            (self.half_road_limit - 1.25) / 3 < obstacle['to_middle'] + 1:
                        ob_to_middle3.append(obstacle['to_middle'])

                else:
                    break

            res = min(len(ob_to_middle1), len(ob_to_middle2), len(ob_to_middle3))
            if res == ob_to_middle1:
                pass
            elif res == ob_to_middle2:
                pass
            else:
                pass

        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}" \
                  .format(car_controls.steering, car_controls.throttle, car_controls.brake))

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
