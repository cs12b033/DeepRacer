
"""
    @author: Ajay Pratap Singh
    @Link: https://github.com/cs12b033/DeepRacer
    @License: GNU Lesser General Public License v3.0
    Model name : reward_RD_001 [Family: RabbitDriver, Gene: Khichdi]
    Model description : This is a model using all params to reward and penalize heavily on mixture of strategies
"""

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
    # print(all_wheels_on_track, x, y, distance_from_center, is_left_of_center, heading, progress, steps, speed, steering_angle, track_width, waypoints, closest_waypoint)

    # Import all libraries related to reward function
    from math import pow, atan2, degrees

    # define variables
    REWARD_MAX = 1e5
    REWARD_MIN = -1e5
    SPEED_MAX = 5
    STEERING_MAX = 30
    SAFE_STEERING_MAX = 13
    SAFE_HEADING_MAX = 10
    YAW_MAX = 180
    STEPS_MAX = 85

    ## Carrots
    reward = 1
    centering_reward = 1
    speed_reward = 1
    waypoints_reward = 1
    heading_reward = 1
    steering_reward = 1
    progress_reward = 1
    very_high_reward = 1e4
    high_reward = 1e3
    medium_reward = 1e2
    low_reward = 1e1

    ## stick
    very_high_penalty = -1e4
    high_penalty = -1e3
    medium_penalty = -1e2
    low_penalty = -1e1

    ## Weights
    central_distance_reward_weight = 4    # legal range [0, 5]
    central_distance_penalty_weight = 4.5     # legal range [0, 5]


    # Markers on track
    marker_0 = 0
    marker_1 = 0.05 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    ##### Progress #######
    if progress >= 100:
        return max(REWARD_MAX/steps, very_high_reward)
    elif steps > 0:
        progress_reward = STEPS_MAX/steps

    ##### Centering ######
    # negative exponential penalty for distance from center
    if distance_from_center > marker_3:
        # print("Marker_3 crossed")
        return 0.4 * REWARD_MIN
    elif distance_from_center > marker_2:
        # print("Marker_2 touched")
        centering_reward = -1 * pow(10, central_distance_penalty_weight * (2*distance_from_center)/track_width)
    else:
        # print("Marker_1 gliding")
        centering_reward = pow(10, central_distance_reward_weight * min(abs(marker_2 - distance_from_center), marker_2)/marker_2)

    ##### (Speed) inversely proportional to (highest time - current time elapsed) [Give less weight] ######
    # speed_reward = pow(10, min(abs(SPEED_MAX - speed), SPEED_MAX)/SPEED_MAX)
    if speed > 4.8:
        speed_reward = pow(10, max(speed/2, 4))
    elif speed > 4.4:
        speed_reward = pow(10, 4 * speed/SPEED_MAX)
    elif speed > 3.5:
        speed_reward = speed/SPEED_MAX
    else:
        speed_reward = -1 * pow(10, abs(4 - speed))

    # # giant penalty if vehicle is exiting track
    if not all_wheels_on_track:
        return very_high_penalty

    # ######################
    # ###### Steering ######
    # ######################
    #
    # penalize reward if orientation of the vehicle deviates way too much when compared to ideal orientation
    next_waypoint = waypoints[closest_waypoint[1]]
    prev_waypoint = waypoints[closest_waypoint[0]]
    heading_diff = abs(degrees(atan2(next_waypoint[1] - prev_waypoint[1], next_waypoint[0] - prev_waypoint[0])) - heading)
    if heading_diff < SAFE_HEADING_MAX:
        heading_reward = high_reward
    else:
        heading_reward = very_high_penalty * heading_diff/STEERING_MAX
    print("waypoint_dir:", degrees(atan2(next_waypoint[1] - prev_waypoint[1], next_waypoint[0] - prev_waypoint[0])))
    print("heading_diff:", heading_diff)

    # penalize reward if the car is steering too much
    if abs(steering_angle) > SAFE_STEERING_MAX:
        steering_reward = 0.75

    print("Each strategy reward: ", centering_reward, speed_reward, heading_reward, steering_reward, waypoints_reward, progress_reward)
    reward = centering_reward + speed_reward + heading_reward + steering_reward + waypoints_reward + progress_reward
    reward = max(min(reward, 1e5), -1e5)
    return float(reward)
