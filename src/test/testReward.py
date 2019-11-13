
import sys
sys.path.append('../')
from rabbitDriver.reward_RD_001 import reward_function as rd1
from rabbitDriver.reward_RD_002 import reward_function as rd2
from rabbitDriver.reward_RD_003 import reward_function as rd3
from rabbitDriver.reward_RD_004 import reward_function as rd4


params = [
{
    "all_wheels_on_track": True,
    "x": 46,
    "y": 12,
    "distance_from_center": 0.59,
    "is_left_of_center": True,
    "heading": -179,
    "progress": 99,
    "steps": 32,
    "speed": 4.9,
    "steering_angle": 29,
    "track_width": 1.6,
    "waypoints": [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18], [19, 20], [21, 22], [23, 24]],
    "closest_waypoints": [3, 4],
},
{
    "all_wheels_on_track": True,
    "x": 0,
    "y": 0,
    "distance_from_center": 0,
    "is_left_of_center": True,
    "heading": 0,
    "progress": 73,
    "steps": 387,
    "speed": 0,
    "steering_angle": 0,
    "track_width": 1.6,
    "waypoints": [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18], [19, 20], [21, 22], [23, 24]],
    "closest_waypoints": [0, 1],
},
{
    "all_wheels_on_track": False,
    "x": 0,
    "y": 0,
    "distance_from_center": 1.8,
    "is_left_of_center": False,
    "heading": 90,
    "progress": 0,
    "steps": 245,
    "speed": 4.5,
    "steering_angle": 29,
    "track_width": 1.6,
    "waypoints": [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18], [19, 20], [21, 22], [23, 24]],
    "closest_waypoints": [1, 2],
},
{
    "all_wheels_on_track": True,
    "x": 6,
    "y": 11,
    "distance_from_center": 0.1,
    "is_left_of_center": True,
    "heading": 50,
    "progress": 35,
    "steps": 26,
    "speed": 4.6,
    "steering_angle": 7,
    "track_width": 1.6,
    "waypoints": [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18], [19, 20], [21, 22], [23, 24]],
    "closest_waypoints": [2, 3],
},
{
    "all_wheels_on_track": True,
    "x": 9,
    "y": 9,
    "distance_from_center": 0.01,
    "is_left_of_center": False,
    "heading": 50,
    "progress": 35,
    "steps": 26,
    "speed": 4.9,
    "steering_angle": 5,
    "track_width": 1.6,
    "waypoints": [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18], [19, 20], [21, 22], [23, 24]],
    "closest_waypoints": [2, 3],
},
{
    "all_wheels_on_track": True,
    "x": 6,
    "y": 7,
    "distance_from_center": 0.1,
    "is_left_of_center": False,
    "heading": 40,
    "progress": 35,
    "steps": 706,
    "speed": 4.3,
    "steering_angle": 9,
    "track_width": 1.6,
    "waypoints": [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18], [19, 20], [21, 22], [23, 24]],
    "closest_waypoints": [2, 3],
},
{
    "all_wheels_on_track": True,
    "x": 7.01,
    "y": 8.01,
    "distance_from_center": 0.01,
    "is_left_of_center": False,
    "heading": 50,
    "progress": 35,
    "steps": 26,
    "speed": 4.9,
    "steering_angle": 5,
    "track_width": 1.6,
    "waypoints": [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18], [19, 20], [21, 22], [23, 24]],
    "closest_waypoints": [3, 4],
},

{
    "all_wheels_on_track": True,
    "x": 7.01,
    "y": 8.01,
    "distance_from_center": 0.01,
    "is_left_of_center": True,
    "heading": 50,
    "progress": 35,
    "steps": 26,
    "speed": 4.9,
    "steering_angle": 5,
    "track_width": 1.6,
    "waypoints": [[1, 2], [3, 4], [5, 6], [7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18], [19, 20], [21, 22], [23, 24]],
    "closest_waypoints": [0, 1],
},
]


def testRewards(rf):

    rewardsList = []
    for thisParam in params:
        rewardsList.append(rf(thisParam))
    print("\nRewards: ", end="")
    for eachReward in rewardsList:
        print(eachReward, end=", ")
    return

if __name__ == "__main__":
    print("Testing rewards")
    i = 1
    functionList = [rd1, rd2, rd3, rd4]
    for eachFunc in functionList:
        print("Function: ", i, eachFunc)
        testRewards(eachFunc)
        print("Ended function", i)
        print("\n\n")
        i += 1
