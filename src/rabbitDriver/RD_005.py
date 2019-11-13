
"""
    @author: Ajay Pratap Singh
    @Link: https://github.com/cs12b033/DeepRacer
    @License: GNU Lesser General Public License v3.0
    Model name : reward_RD_005 [Family: RabbitDriver, Gene: Punisher]
    Model description : Minimalistic functions to give rewards for being on track and punishing heavily if off-track
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
        reward = 1

        # TODO: Write the logic for RD_005


        print("Reward::", reward)
        return float(check_reward_bounds(reward))
    except Exception as e:
        print("Exception: ", e.__str__())
        return float(check_reward_bounds(-1e2))
