
"""
    @author: Ajay Pratap Singh
    @Link: https://github.com/cs12b033/DeepRacer
    @License: GNU Lesser General Public License v3.0
    Model name : reward_RD_008 [Family: RabbitDriver, Gene: DedicatedFollower]
    Model description : Follows center line
"""


def check_reward_bounds(reward):
    """

    :param reward:
    :return:
    """
    return max(min(reward, 1e5), -1e5)


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
            # all_wheels_on_track = params['all_wheels_on_track']
            # x = params['x']
            # y = params['y']
            distance_from_center = params['distance_from_center']
            # is_left_of_center = params['is_left_of_center']
            # heading = params['heading']
            # progress = params['progress']
            # steps = params['steps']
            speed = params['speed']
            steering_angle = params['steering_angle']
            track_width = params['track_width']
            # waypoints = params['waypoints']
            # closest_waypoint = params['closest_waypoints']
            # is_reversed = params["is_reversed"]
        except Exception as e:
            # is_reversed = False
            # print("Exception:", e.__str__())
            pass
        print("\n\tParams:: {", end="")
        for key in params.keys():
            print(key, ":", params[key], end=", ")
        print("}")

        #
        SPEED_MAX = 5
        SAFE_STEERING = 15
        STEERING_MAX = 30

        # Calculate 3 markers that are at varying distances away from the center line
        marker_1 = 0.1 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.5 * track_width

        # Give higher reward if the car is closer to center line and vice versa
        if distance_from_center <= marker_1:
            reward = 5.0
        elif distance_from_center <= marker_2:
            reward = 0.5
        elif distance_from_center <= marker_3:
            reward = 0.1
        else:
            reward = 1e-3  # likely crashed

        reward += pow(0.6 + speed/SPEED_MAX, 4)

        if steering_angle > SAFE_STEERING:
            reward *= (1 - 0.1 * steering_angle/STEERING_MAX)

        print("Reward::", reward)
        return float(check_reward_bounds(reward))
    except Exception as e:
        print("Exception: ", e.__str__())
        return float(check_reward_bounds(-1e2))
