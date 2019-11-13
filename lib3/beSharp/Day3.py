import math
def reward_function(params):
    '''
        Use square root for center line
    '''
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    speed = params['speed']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    SPEED_TRESHOLD = 6
    reward = 1 - (distance_from_center / (track_width/2))**(4)
    if reward < 0:
        reward = 0
    if speed < SPEED_TRESHOLD:
        reward *= 0.8
    if not (all_wheels_on_track):
        reward = 0
    if progress == 100:
        reward += 100
    return float(reward)