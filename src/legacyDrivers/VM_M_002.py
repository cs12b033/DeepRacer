"""
    @author : Ajay Pratap Singh
    VM-M-002 [Family: LegacyDrivers, Gene: SlowNSteady ]
    @description: Minimalistic approach to take preferably slow and steady actions
"""
def reward_function(params):
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    all_wheels_on_track = params['all_wheels_on_track']
    speed = params['speed']
    SPEED_MIN = 1.5
    SPEED_MAX = 5.0

    dead_center_marker = 0.1 * track_width
    preferred_margin_marker = 0.25 * track_width
    safe_marker = 0.4 * track_width
    half_track_marker = 0.5 * track_width

    if distance_from_center <= dead_center_marker:
        reward = 1.0
    elif distance_from_center <= preferred_margin_marker:
        reward = 0.6
    elif distance_from_center <= safe_marker:
        reward = 1e-1
    elif distance_from_center <= half_track_marker:
        reward = 1e-2
    else:
        reward = 1e-3

    if not all_wheels_on_track:
        # Penalize if the car goes off track
        return 1e-3
    elif speed < SPEED_MIN:
        # Penalize if the car goes too slow
        reward += 1e-1
    else:
        # High reward if the car stays on track and goes fast
        reward += speed/SPEED_MAX

    return float(reward)
