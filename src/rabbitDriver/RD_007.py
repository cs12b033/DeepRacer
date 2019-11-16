
"""
    @author: Ajay Pratap Singh
    @Link: https://github.com/cs12b033/DeepRacer
    @License: GNU Lesser General Public License v3.0
    Model name : reward_RD_007 [Family: RabbitDriver, Gene: Simple]
    Model description : Simple model to reward good actions and penalize bad ones
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
        reward = 1
    elif distance_from_center <= preferred_margin_marker:
        reward = 0.6
    elif distance_from_center <= safe_marker:
        reward = 5 * 1e-2
    elif distance_from_center <= half_track_marker:
        reward = 1e-3
    else:
        reward = REWARD_MIN
    return reward

def progress_reward(progress, steps):
    if progress/100 > steps/STEPS_IDEAL:
        reward = 2 * progress/(steps+1e-3)
    else:
        reward = 0.1 * progress/(steps+1e-3)
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


        # Penalize Off Track
        if not all_wheels_on_track:
            return REWARD_MIN

        # Penalize if reversed
        if is_reversed:
            return REWARD_MIN

        reward = 0

        # Penalize frozen speed
        if speed < SPEED_MIN:
            return REWARD_MIN
        else:
            reward += speed/SPEED_MAX

        reward += centering_reward(track_width, distance_from_center)

        reward += progress_reward(progress, steps)

        # Penalize too much steering
        reward *= steering_reward(steering_angle)

        print("Reward::", reward)
        return float(check_reward_bounds(reward))
    except Exception as e:
        print("Exception: ", e.__str__())
        return float(check_reward_bounds(-1e2))
