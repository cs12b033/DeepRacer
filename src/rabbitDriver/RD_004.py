
"""
    @author: Ajay Pratap Singh
    @Link: https://github.com/cs12b033/DeepRacer
    @License: GNU Lesser General Public License v3.0
    Model name : reward_RD_004 [Family: RabbitDriver, Gene: Ghoda]
    Model description : This is a model using all params to calculate reward by normalizing each param values, then multiplying them, and penalizing heavily for wrong
"""

def check_reward_bounds(reward):
    """

    :param reward:
    :return:
    """
    return max(min(reward, 1e5), -1e5)

def calculate_reward(progress_reward, centering_reward, speed_reward, waypoints_reward, heading_reward, steering_reward, left_reward, penalty):
    STRATEGY_REWARD_COEFFICIENT = 1
    STRATEGY_REWARD_POWER = 1
    rewards = \
            pow(progress_reward, STRATEGY_REWARD_POWER) * \
            pow(centering_reward, 2) * \
            pow(speed_reward, 1.5) * \
            pow(waypoints_reward, 1.1) * \
            pow(heading_reward, STRATEGY_REWARD_POWER) * \
            pow(steering_reward, STRATEGY_REWARD_POWER) * \
            pow(left_reward, STRATEGY_REWARD_POWER)
    return STRATEGY_REWARD_COEFFICIENT  * rewards + penalty

def reward_function(params):
    """
        @:param
            "all_wheels_on_track": Boolean,    # flag to indicate if the vehicle is on the track
            "x": float, [0, INF)                       # vehicle's x-coordinate in meters
            "y": float, [0, INF)                       # vehicle's y-coordinate in meters
            "distance_from_center": float, [0, 0.6), where 0.6 represents (track_width/2 - width_of_car) )   # distance in meters from the track center
            "is_left_of_center": Boolean,      # Flag to indicate if the vehicle is on the left side to the track center or not.
            "heading": float, [-180, 180]                 # vehicle's yaw in degrees
            "progress": float, [0, 100]                # percentage of track completed
            "steps": int, [0, INF)                     # number steps completed
            "speed": float, [0, 5]                    # vehicle's speed in meters per second (m/s)
            "steering_angle": float, [-30, 30]         # vehicle's steering angle in degrees
            "track_width": float, [0, 1.6)             # width of the track
            "waypoints": [[float, float], … ], # list of [x,y] as milestones along the track center
            "closest_waypoints": [int, int]    # indices of the two nearest waypoints.
        @:returns
            reward : float [-1e5, 1e5]
    """
    # TODO:
    # * Make proper logs

    try:

        # Import all libraries related to reward function
        from math import pow, atan2, degrees

        # initialized variables from param
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
        closest_waypoint = params['closest_waypoints']
        print("\n\tParams:: {", end="")
        for key in params.keys():
            print(key, ":", params[key], end=", ")
        print("}")

        # define variables
        REWARD_MAX = 1e5
        REWARD_MIN = -1e5
        SPEED_MAX = 5
        STEERING_MAX = 30
        STEPS_PENALTY_RATE = -100
        STEPS_IDEAL1 = 90
        STEPS_IDEAL2 = 300
        STEPS_MAX = 90
        STEPS_REWARD_OFFSET2 = 5 * 1e3
        SAFE_STEERING_MAX = 15
        SAFE_HEADING_MAX = 10
        YAW_MAX = 180
        STEERING_ANGLE_WEIGHT_1 = 0.5


        reward = 1
        penalty = 0

        # rewards                           # range (0, ~6]
        centering_reward = 1
        speed_reward = 1
        waypoints_reward = 1
        heading_reward = 1
        steering_reward = 1
        progress_reward = 1
        left_reward = 1

        ## Carrots
        very_high_reward = 1e4
        high_reward = 1e3
        medium_reward = 1e2
        low_reward = 1e1
        non_zero_reward = 1e-3
        almost_zero_reward =1e-5

        ## stick
        very_high_penalty = -1e4
        high_penalty = -1e3
        medium_penalty = -1e2
        low_penalty = -1e1

        ## Weights
        central_distance_reward_weight = 4  # legal range [0, 5]
        central_distance_penalty_weight = 4.5  # legal range [0, 5]
        central_distance_weight_2 = 1

        # Markers on track
        marker_0 = 0
        marker_1 = 0.05 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.5 * track_width

        # ######################
        # ###### PROGRESS ######
        # ######################
        if progress >= 100:
            # reward = max(REWARD_MAX/(0.1*steps), very_high_reward)
            if steps < STEPS_IDEAL1:
                reward = very_high_reward + pow(STEPS_MAX - steps, 2) + STEPS_REWARD_OFFSET2
            elif steps < STEPS_IDEAL2:
                reward = very_high_reward + (STEPS_PENALTY_RATE * steps) + STEPS_REWARD_OFFSET2
            else:
                reward = very_high_reward + -0.1 * pow(steps, 2)
            return check_reward_bounds(reward)
        elif progress > (steps / STEPS_IDEAL1) * 100:
            progress_reward = 5
        else:
            progress_reward = 1 + progress/(steps+0.01)

        # ######################
        # ###### ON_TRACK ######
        # ######################
        # # giant penalty if vehicle is exiting track
        if not all_wheels_on_track:
            print("Penalty:: Vehicle not on track ", all_wheels_on_track)
            return check_reward_bounds(very_high_penalty)


        # ######################
        # ###### IS_LEFT ######
        # ######################
        if is_left_of_center:
            left_reward = 1.2


        # ######################
        # ###### WAYPOINTS ######
        # ######################
        N1 = waypoints[closest_waypoint[1]]
        N3 = waypoints[closest_waypoint[1] + 2]
        N5 = waypoints[closest_waypoint[1] + 4]
        N7 = waypoints[closest_waypoint[1] + 6]
        P = waypoints[closest_waypoint[0]]
        # NY, NX = N[1], N[0]
        PY, PX = P[1], P[0]
        N1Y, N1X = N1[1], N1[0]
        N3Y, N3X = N3[1], N3[0]
        N5Y, N5X = N5[1], N5[0]
        N7Y, N7X = N7[1], N7[0]
        # print(x, y)
        # print(PX, PY)
        # print(N3X, N3Y)
        waypoints_dist_N1P = pow(pow((N1Y - PY), 2) + pow((N1X - PX), 2), 0.5)
        waypoints_dist_CN1 = pow(pow((y - N1Y), 2) + pow((x - N1X), 2), 0.5)
        waypoints_dist_N3P = pow(pow((N3Y - PY), 2) + pow((N3X - PX), 2), 0.5)
        waypoints_dist_CN3 = pow(pow((y - N3Y), 2) + pow((x - N3X), 2), 0.5)
        # waypoints_dist_CN5 = pow(pow((y - N5Y), 2) + pow((x - N5X), 2), 0.5)
        # waypoints_dist_CN7 = pow(pow((y - N7Y), 2) + pow((x - N7X), 2), 0.5)
        waypoints_dist_CP = pow(pow((y - PY), 2) + pow((x - PX), 2), 0.5)
        # waypoints_reward =  max(waypoints_dist_CN + waypoints_dist_CP - waypoints_dist_NP
        # waypoint_relative_position_offset = ((waypoints_dist_CN3 + waypoints_dist_CP) /waypoints_dist_N3P - 1)*100
        # print("waypoint_relative_position_offset: ", waypoint_relative_position_offset)
        distA = waypoints_dist_N3P/2
        distB = track_width/2
        distC = pow(pow(distA, 2) + pow(distB, 2), 0.5)
        print("Info:: distC:", distC, "waypoints_dist_CN3:", waypoints_dist_CN3, "waypoints_dist_CP:", waypoints_dist_CP)
        # if waypoint_relative_position_offset <= 0:
        #     print("ERROR: Impossible situation!", waypoints_dist_N3P, waypoints_dist_CN3, waypoints_dist_CP)
        #     waypoints_reward = 1        # Not sure how to evaluate impossible situations
        # elif waypoint_relative_position_offset == 0:
        #     # It is on the shortest path between 3rd next waypoint and previous waypoint. Give good reward
        #     waypoints_reward = 2.5 * low_reward
        if waypoints_dist_CN3 + waypoints_dist_CP <= 2 * distC:
            if (waypoints_dist_CN1 + waypoints_dist_CP)/waypoints_dist_N1P > 1.3:
                print("Penalty:: Vehicle seems to have gone far from waypoints (Penalty#21352),", waypoints_dist_CN1, waypoints_dist_CP)
                waypoints_reward = waypoints_dist_N1P/(waypoints_dist_CN1 + waypoints_dist_CP)
                penalty += low_penalty
            else:
                # print("OK:: Waypoints", waypoints_dist_CN1, waypoints_dist_CP, waypoints_dist_N1P)
                waypoints_reward = 2 * (2*distC)/(waypoints_dist_CN3 + waypoints_dist_CP)
        else:
            print("Penalty:: Vehicle seems to have gone far from waypoints (Penalty#46737)", distC, waypoints_dist_CN3, waypoints_dist_CP)
            penalty += low_penalty
            waypoints_reward = almost_zero_reward

        # ######################
        # ###### STEERING ######
        # ######################
        # penalize reward if the car is steering too much
        if abs(steering_angle) <= SAFE_STEERING_MAX:
            steering_reward = 1 + (SAFE_STEERING_MAX - steering_angle)/SAFE_STEERING_MAX
        else:
            print("Penalty:: Steering too much", steering_angle)
            steering_reward = non_zero_reward
            penalty += low_penalty*steering_angle/SAFE_STEERING_MAX

        # ######################
        # ###### HEADING ######
        # ######################
        #
        # penalize reward if orientation of the vehicle deviates way too much when compared to ideal orientation
        # next_waypoint = waypoints[closest_waypoint[1]]
        # prev_waypoint = waypoints[closest_waypoint[0]]
        # dirN1C : 'next 1 waypoint' from current position
        dirN1C = degrees(atan2(abs(N1Y - y), abs(N1X - x)))
        # dirN1P1 : 'next 1 waypoint' from 'previous 1 waypoint'
        # dirN1P1 = degrees(atan2(next_waypoint[1] - prev_waypoint[1], next_waypoint[0] - prev_waypoint[0]))
        # dirN3C : 'next 3 waypoint' from current position
        dirN3C = degrees(atan2(abs(N3Y - y), abs(N3X - x)))
        # dirN5C : 'next 5 waypoint' from current position
        dirN5C = degrees(atan2(abs(N5Y - y), abs(N5X - x)))
        # dirN7C : 'next 7 waypoint' from current position
        dirN7C = degrees(atan2(abs(N7Y - y), abs(N7X - x)))
        print("Info:: WayP Dir: ", dirN1C, dirN3C, dirN5C, dirN7C)
        heading1_diff = abs(heading - STEERING_ANGLE_WEIGHT_1 * steering_angle - dirN1C)
        heading3_diff = abs(heading - STEERING_ANGLE_WEIGHT_1 * steering_angle - dirN3C)
        print("Info:: heading: ", heading, "heading1_diff:", heading1_diff, "heading3_diff:", heading3_diff)
        if heading1_diff <= SAFE_HEADING_MAX and heading3_diff <= SAFE_HEADING_MAX:
            heading_reward = min((2 + (SAFE_HEADING_MAX - heading1_diff)/SAFE_HEADING_MAX), 2.4)
        elif heading1_diff <= SAFE_HEADING_MAX:
            heading_reward = min((1 + (SAFE_HEADING_MAX - heading1_diff) / SAFE_HEADING_MAX), 1.2)
        else:
            print("Penalty:: heading not in safe direction,", heading1_diff, heading3_diff)
            heading_reward = non_zero_reward
            penalty += low_penalty
            # give more reward to centering_reward if within bounds, to compensate for wrong heading
            if distance_from_center < marker_2:
                centering_reward += 0.5

        # ######################
        # ###### SPEED ######
        # ######################
        safeDirN7C = True if abs(heading - STEERING_ANGLE_WEIGHT_1 * steering_angle - dirN7C) <= SAFE_HEADING_MAX else False
        safeDirN5C = True if abs(heading - STEERING_ANGLE_WEIGHT_1 * steering_angle - dirN5C) <= SAFE_HEADING_MAX else False
        safeDirN3C = True if abs(heading - STEERING_ANGLE_WEIGHT_1 * steering_angle - dirN3C) <= SAFE_HEADING_MAX else False
        safeDirN1C = True if abs(heading - STEERING_ANGLE_WEIGHT_1 * steering_angle - dirN1C) <= SAFE_HEADING_MAX else False
        print("Info::", safeDirN1C, safeDirN3C, safeDirN5C, safeDirN7C)
        if speed > 4:
            if safeDirN7C and safeDirN3C and safeDirN1C:
                speed_reward = 4 * (speed+1)
            elif safeDirN5C and safeDirN3C and safeDirN1C:
                speed_reward = 3 * (speed+1)
            elif safeDirN3C and safeDirN1C:
                speed_reward = 2 * speed
            elif safeDirN1C:
                speed_reward = 0.9
            else:
                print("Info:: Needed sharp turn, slow down speed preferred")
                speed_reward = 0.1 * max(abs(SPEED_MAX - speed), 4)
        elif speed > 3.3:
            if safeDirN1C and not safeDirN3C:
                speed_reward = speed / 3
            else:
                speed_reward = 2 + speed/SPEED_MAX
        elif speed > 2.1:
            if safeDirN1C and not safeDirN3C:
                speed_reward = 2 * speed
            else:
                speed_reward = 1 + speed/SPEED_MAX
                print("Penalty:: Low speed,", speed)
                penalty += low_penalty
        elif speed > 1:
            if safeDirN1C and not safeDirN3C:
                speed_reward = 5
            else:
                speed_reward = non_zero_reward
                print("Penalty:: Low Speed,", speed)
                penalty += high_penalty
        else:
            speed_reward = non_zero_reward
            print("Penalty:: Low Speed,", speed)
            penalty += high_penalty

        # ######################
        # ###### CENTERING ######
        # ######################
        # negative exponential penalty for distance from center
        if distance_from_center > marker_3:
            print("Penalty:: Marker_3 crossed, ", distance_from_center)
            return check_reward_bounds(0.4 * REWARD_MIN)
        elif distance_from_center > marker_2:
            print("Penalty:: Marker_2 crossed, ", distance_from_center)
            if safeDirN1C or safeDirN3C:
                centering_reward += 2
            else:
                centering_reward = almost_zero_reward
                penalty += medium_penalty
        else:
            # print("OK:: Inside Marker_2")
            if marker_2 < 1:
                central_distance_weight_2 = 1 / marker_2
            centering_reward += 3 * central_distance_weight_2 * (marker_2 - distance_from_center)

        # Finalize
        print("Info:: Progress_reward:", progress_reward, ", centering_reward:", centering_reward, ", speed_reward:", speed_reward, ", waypoints_reward:", waypoints_reward, ", heading_reward:", heading_reward, ", steering_reward:", steering_reward, ", left_reward:", left_reward, ", Penalty:", penalty)
        reward = calculate_reward(progress_reward, centering_reward, speed_reward, waypoints_reward, heading_reward, steering_reward, left_reward, penalty)
        print("Reward::", reward)
        return float(reward)
    except Exception as e:
        print("Exception: ", e.__str__())
        return float(check_reward_bounds(-1e2))
