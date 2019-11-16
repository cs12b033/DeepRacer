
"""
    @author: Ajay Pratap Singh
    @Link: https://github.com/cs12b033/DeepRacer
    @License: GNU Lesser General Public License v3.0
    Model name : reward_RD_006 [Family: RabbitDriver, Gene: WayPoint]
    Model description : Heavily relying on Waypoints and modulating steering, speed, heading according to that. Could be a disaster on real-track
"""

# Import all libraries related to reward function
from math import pow, atan2, degrees

# GLOBAL CONSTANTS
REWARD_MAX = 1e5
REWARD_MIN = -1e5
SPEED_MAX = 5
SPEED_MIN = 1.2
STEERING_MAX = 30
STEPS_IDEAL = 90
SAFE_HEADING_MAX = 12


def check_reward_bounds(reward):
    """

    :param reward:
    :return:
    """
    return max(min(reward, 1e5), -1e5)


def waypoint_reward(speed, nextWP, is_left_of_center, inBounds):
    if inBounds:
        if (0 <= nextWP and nextWP <= 8) or \
            (25 <= nextWP and nextWP <= 28) or \
            (34 <= nextWP and nextWP <= 39) or \
            (46 <= nextWP and nextWP <= 49) or \
            (54 <= nextWP and nextWP <= 60) or \
            (69 <= nextWP and nextWP <= 70):
            # Straight : High Speed
            reward = pow(2 * speed / SPEED_MAX, 3)
            if is_left_of_center:
                reward *= 1.1
            else:
                reward *= 0.95
        elif (9 <= nextWP and nextWP <= 24) or \
            (29 <= nextWP and nextWP <= 33) or \
            (40 <= nextWP and nextWP <= 45) or \
            (50 <= nextWP and nextWP <= 53) or \
            (61 <= nextWP and nextWP <= 68):
            # Curve : Slow speed
            reward = pow(3 * (1 - speed / SPEED_MAX), 3)
            if 29 <= nextWP and nextWP <= 33:
                if not is_left_of_center:
                    reward *= 1.1
                else:
                    reward *= 0.95
            else:
                if is_left_of_center:
                    reward *= 1.1
                else:
                    reward *= 0.95
        else:
            reward = 1e-3
    else:
        reward = 1e-3
    return reward

def steering_reward(steering_angle):
    # Steering penality threshold, change the number based on your action space setting
    ABS_STEERING_THRESHOLD = 15
    # Penalize reward if the agent is steering too much
    if steering_angle > ABS_STEERING_THRESHOLD:
        reward = 0.8
    else:
        reward = 1
    return reward

def centering_reward(track_width, distance_from_center):
    dead_center_marker = 0.1 * track_width
    preferred_margin_marker = 0.25 * track_width
    safe_marker = 0.4 * track_width
    half_track_marker = 0.5 * track_width

    if distance_from_center <= dead_center_marker:
        reward = 10
    elif distance_from_center <= preferred_margin_marker:
        reward = 6
    elif distance_from_center <= safe_marker:
        reward = 1
    elif distance_from_center <= half_track_marker:
        reward = 1e-2
    else:
        reward = REWARD_MIN
    return reward

def progress_reward(progress, steps):
    if progress/100 > steps/STEPS_IDEAL:
        reward = progress/(steps+1e-3)
    else:
        reward = 1e-3
    return reward

def heading_reward(heading, steering_angle, WP_N1, WP_N2, x, y):
    reward = 0
    N1Y, N1X = WP_N1[1], WP_N1[0]
    N2Y, N2X = WP_N2[1], WP_N2[0]
    dirN2C = degrees(atan2(abs(N2Y - y), abs(N2X - x)))
    dirN1C = degrees(atan2(abs(N1Y - y), abs(N1X - x)))
    safeDirN1C = True if abs(heading - steering_angle - dirN1C) <= SAFE_HEADING_MAX else False
    safeDirN2C = True if abs(heading - steering_angle - dirN2C) <= SAFE_HEADING_MAX else False
    if safeDirN1C and safeDirN2C:
        reward = 1
    elif safeDirN1C and not safeDirN2C:
        reward = 0.3
    else:
        reward = 1e-3
    return reward

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
            "is_reversed": Boolean,             # Unknown. Default: False. Probably to check if the vehicle is reversed
        @:returns
            reward : float [-1e5, 1e5]
    """
    # TODO:
    # * Make proper logs

    try:


        # initialized variables from param
        try:
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
            is_reversed = params["is_reversed"]
        except Exception as e:
            is_reversed = False
            # print("Exception:", e.__str__())
        print("\n\tParams:: {", end="")
        for key in params.keys():
            print(key, ":", params[key], end=", ")
        print("}")

        reward = 0
        if distance_from_center / track_width < 0.4:
            inBounds = True
        else:
            inBounds = False
        nextWP_index = closest_waypoint[1]

        # Penalize Off Track
        if not all_wheels_on_track:
            return REWARD_MIN

        # Penalize if reversed
        if is_reversed:
            return REWARD_MIN

        reward += centering_reward(track_width, distance_from_center)

        reward += waypoint_reward(speed, nextWP_index, is_left_of_center, inBounds)

        # Penalize frozen speed
        if speed < SPEED_MIN:
            return REWARD_MIN

        # Penalize too much steering
        reward *= steering_reward(steering_angle)

        reward += progress_reward(progress, steps)

        reward += heading_reward(heading, steering_angle, waypoints[nextWP_index], waypoints[nextWP_index + 1], x, y)

        print("Reward::", reward)
        return float(check_reward_bounds(reward))
    except Exception as e:
        print("Exception: ", e.__str__())
        return float(check_reward_bounds(-1e2))
