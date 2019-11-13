
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
            pow(centering_reward, STRATEGY_REWARD_POWER) * \
            pow(speed_reward, 1.5) * \
            pow(waypoints_reward, 1.5) * \
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
    # NOTES:
    # * Keep most of the rewards in range (-1e4, 1e4)
    # * Reserve -1e5 and 1e5 for leaving track and race successfully complete
    # * Decide weather to Multiply multiple strategy rewards or Add them
    #

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
        print("\n\tParams: >>>>", end="")
        for key in params.keys():
            print("{", key, ": ", params[key], "}, ", end="")
        print("<<<<")

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
            print("Penalty: Vehicle not on track ", all_wheels_on_track)
            return check_reward_bounds(very_high_penalty)

        # ######################
        # ###### CENTERING ######
        # ######################
        # negative exponential penalty for distance from center
        if distance_from_center > marker_3:
            print("Penalty: Marker_3 crossed, ", distance_from_center)
            return check_reward_bounds(0.4 * REWARD_MIN)
        elif distance_from_center > marker_2:
            print("Penalty: Marker_2 crossed, ", distance_from_center)
            # centering_reward = -1 * pow(10, central_distance_penalty_weight * (2*distance_from_center)/track_width)
            centering_reward = 0
            penalty += medium_penalty
        else:
            print("Within Marker_2")
            # centering_reward = pow(10, central_distance_reward_weight * min(abs(marker_2 - distance_from_center), marker_2)/marker_2)
            centering_reward = 2

        # ######################
        # ###### IS_LEFT ######
        # ######################
        if is_left_of_center:
            left_reward = 1.2

        # ######################
        # ###### SPEED ######
        # ######################
        if speed > 3.8:
            speed_reward = max(min(6*(speed-3.8), 5), 9)
        else:
            speed_reward = non_zero_reward
            print("Penalty: Low Speed, ", speed)
            penalty += low_penalty

        # ######################
        # ###### WAYPOINTS ######
        # ######################
        # N = waypoints[closest_waypoint[1]]
        N3 = waypoints[closest_waypoint[1] + 2]
        P = waypoints[closest_waypoint[0]]
        # NY, NX = N[1], N[0]
        PY, PX = P[1], P[0]
        N3Y, N3X = N3[1], N3[0]
        # print(x, y)
        # print(PX, PY)
        # print(N3X, N3Y)
        # waypoints_dist_NP = pow(pow((NY - PY), 2) + pow((NX - PX), 2), 0.5)
        # waypoints_dist_CN = pow(pow((y - NY), 2) + pow((x - NX), 2), 0.5)
        waypoints_dist_N3P = pow(pow((N3Y - PY), 2) + pow((N3X - PX), 2), 0.5)
        waypoints_dist_CN3 = pow(pow((y - N3Y), 2) + pow((x - N3X), 2), 0.5)
        waypoints_dist_CP = pow(pow((y - PY), 2) + pow((x - PX), 2), 0.5)
        # waypoints_reward =  max(waypoints_dist_CN + waypoints_dist_CP - waypoints_dist_NP
        # waypoint_relative_position_offset = ((waypoints_dist_CN3 + waypoints_dist_CP) /waypoints_dist_N3P - 1)*100
        # print("waypoint_relative_position_offset: ", waypoint_relative_position_offset)
        distA = waypoints_dist_N3P/2
        distB = track_width/2
        distC = pow(pow(distA, 2) + pow(distB, 2), 0.5)
        # if waypoint_relative_position_offset <= 0:
        #     print("ERROR: Impossible situation!", waypoints_dist_N3P, waypoints_dist_CN3, waypoints_dist_CP)
        #     waypoints_reward = 1        # Not sure how to evaluate impossible situations
        # elif waypoint_relative_position_offset == 0:
        #     # It is on the shortest path between 3rd next waypoint and previous waypoint. Give good reward
        #     waypoints_reward = 2.5 * low_reward
        if 2 * distC <= waypoints_dist_CN3 + waypoints_dist_CP:
            waypoints_reward = 20/(waypoints_dist_CN3 + waypoints_dist_CP)
        else:
            print("Penalty: Vehicle seems to have gone far from waypoints", distC, waypoints_dist_CN3, waypoints_dist_CP)
            penalty += low_penalty
            waypoints_reward = almost_zero_reward

        # ######################
        # ###### STEERING ######
        # ######################
        # penalize reward if the car is steering too much
        if abs(steering_angle) <= SAFE_STEERING_MAX:
            steering_reward = 1 + (SAFE_STEERING_MAX - steering_angle)/SAFE_STEERING_MAX
        else:
            print("Penalty: Steering too much", steering_angle)
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
        # dirN1C = degrees(atan2(next_waypoint[1] - y, next_waypoint[0] - x))
        # dirN1P1 : 'next 1 waypoint' from 'previous 1 waypoint'
        # dirN1P1 = degrees(atan2(next_waypoint[1] - prev_waypoint[1], next_waypoint[0] - prev_waypoint[0]))
        # dirN3C : 'next 3 waypoint' from current position
        dirN3C = degrees(atan2(waypoints[closest_waypoint[1] + 2][1] - y, waypoints[closest_waypoint[1] + 2][0]  - x))
        # dirN5C : 'next 5 waypoint' from current position
        # dirN5C = degrees(atan2(waypoints[closest_waypoint[1] + 4][1] - y, waypoints[closest_waypoint[1] + 4][0]  - x))
        # print(dirN1C, dirN1P1, dirN3C, dirN5C)
        heading_diff = abs(dirN3C - heading)
        print("dirN3C: ", dirN3C, "heading: ", heading, "heading_diff:", heading_diff)
        if heading_diff <= SAFE_HEADING_MAX:
            heading_reward = (SAFE_HEADING_MAX - heading_diff)/SAFE_HEADING_MAX
        else:
            print("Penalty: heading not in safe direction,", heading_diff)
            heading_reward = non_zero_reward
            penalty += low_penalty

        # Finalize
        print("Progress_reward:", progress_reward, ", centering_reward:", centering_reward, ", speed_reward:", speed_reward, ", waypoints_reward:", waypoints_reward, ", heading_reward:", heading_reward, ", steering_reward:", steering_reward, ", left_reward:", left_reward, ", Penalty:", penalty)
        reward = calculate_reward(progress_reward, centering_reward, speed_reward, waypoints_reward, heading_reward, steering_reward, left_reward, penalty)
        print("Reward: ", reward)
        return float(reward)
    except Exception as e:
        print("Exception: ", e.__str__())
        return float(check_reward_bounds(-1e2))