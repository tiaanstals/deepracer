import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5



        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]


        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            
            try:
                if end < start:
                    end += array_len
            except:
                print('error cyclical issue')

            try:
                return [index % array_len for index in range(start, end)]
            except:
                return []

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[2.89403, 0.70184, 2.42138, 0.12804],
                        [3.16466, 0.693, 2.5, 0.10831],
                        [3.43314, 0.68823, 2.212, 0.12139],
                        [3.73805, 0.68548, 1.74133, 0.17511],
                        [4.10749, 0.68438, 1.47974, 0.24967],
                        [4.41121, 0.68403, 1.30858, 0.2321],
                        [4.70859, 0.68388, 1.18364, 0.25125],
                        [5.32, 0.68405, 1.08748, 0.56223],
                        [5.47294, 0.68837, 1.01051, 0.1514],
                        [5.73669, 0.70621, 0.94687, 0.27919],
                        [5.99188, 0.74397, 0.89113, 0.28949],
                        [6.2125, 0.80009, 0.89113, 0.25545],
                        [6.40159, 0.8723, 0.86911, 0.2329],
                        [6.56417, 0.95947, 0.84063, 0.21945],
                        [6.70337, 1.06122, 0.84063, 0.20512],
                        [6.8203, 1.17773, 0.83, 0.19887],
                        [6.91413, 1.30941, 0.83, 0.19481],
                        [6.98181, 1.45656, 0.83, 0.19514],
                        [7.02175, 1.61695, 0.83, 0.19914],
                        [7.02831, 1.78832, 0.83, 0.20663],
                        [6.99394, 1.9655, 0.83, 0.21744],
                        [6.91406, 2.13948, 0.83, 0.23065],
                        [6.78253, 2.29682, 1.00565, 0.20393],
                        [6.6155, 2.43265, 1.12138, 0.19198],
                        [6.42189, 2.544, 1.28767, 0.17345],
                        [6.21112, 2.6322, 1.55951, 0.14651],
                        [5.99094, 2.70254, 1.50459, 0.15362],
                        [5.76663, 2.76273, 1.50459, 0.15435],
                        [5.56291, 2.81599, 1.50459, 0.13995],
                        [5.36026, 2.87264, 1.50459, 0.13985],
                        [5.15931, 2.93486, 1.50459, 0.13981],
                        [4.96058, 3.00487, 1.50459, 0.14004],
                        [4.76448, 3.08511, 1.50459, 0.14082],
                        [4.57237, 3.18404, 1.68304, 0.12839],
                        [4.38341, 3.29902, 1.94311, 0.11384],
                        [4.19707, 3.42683, 1.78142, 0.12684],
                        [4.01268, 3.56362, 1.62656, 0.14115],
                        [3.82932, 3.70508, 1.5121, 0.15315],
                        [3.67912, 3.81836, 1.42056, 0.13244],
                        [3.52833, 3.92723, 1.34911, 0.13786],
                        [3.37653, 4.02968, 1.28679, 0.14232],
                        [3.22319, 4.12407, 1.23044, 0.14634],
                        [3.06756, 4.20907, 1.17802, 0.15053],
                        [2.9086, 4.28361, 1.13048, 0.15531],
                        [2.74479, 4.34671, 1.086, 0.16164],
                        [2.57382, 4.39716, 1.03038, 0.173],
                        [2.39203, 4.43305, 0.95755, 0.19351],
                        [2.19309, 4.45055, 0.95755, 0.20856],
                        [1.96584, 4.44093, 0.95755, 0.23753],
                        [1.70032, 4.38487, 0.95755, 0.28341],
                        [1.42034, 4.26099, 0.95755, 0.31973],
                        [1.16503, 4.06146, 0.95755, 0.3384],
                        [0.96753, 3.78363, 0.95755, 0.35598],
                        [0.87363, 3.43687, 1.24518, 0.28851],
                        [0.85453, 3.09651, 1.42876, 0.2386],
                        [0.8766, 2.81168, 1.27334, 0.22436],
                        [0.91229, 2.57756, 1.1584, 0.20445],
                        [0.96294, 2.31103, 1.06885, 0.25382],
                        [1.00825, 2.10289, 0.99587, 0.2139],
                        [1.0623, 1.90085, 0.99587, 0.21001],
                        [1.12998, 1.70432, 0.99587, 0.20872],
                        [1.21209, 1.52228, 0.99587, 0.20053],
                        [1.30759, 1.3607, 0.99587, 0.18847],
                        [1.41609, 1.22064, 0.99587, 0.1779],
                        [1.53931, 1.10095, 0.99587, 0.1725],
                        [1.68365, 1.00024, 1.15557, 0.15231],
                        [1.85113, 0.91238, 1.25962, 0.15014],
                        [2.04923, 0.83633, 1.39621, 0.15198],
                        [2.28992, 0.77293, 1.58775, 0.15676],
                        [2.58494, 0.72608, 1.88181, 0.15874]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']


        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        # Make sure staying on track is rewarded very highly 
        if not is_offtrack:
            reward += 10
        else:
            reward -= 20

        
        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 5
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 3
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE


        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = -7
        elif direction_diff < 20 and all_wheels_on_track:
            reward += 10
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = -2
        elif speed - optimals[2] and (not is_offtrack):
            reward += 10
        

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 2000 # should be adapted to track length and other rewards
        STANDARD_TIME = 15  # seconds (time that is easily done by model)
        FASTEST_TIME = 8  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward

        # progress reward
        # give out an additional 2k for progress - gives very little over the first 75%
        var_a = 36000/(100**9)
        reward += var_a*(progress**8)

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)