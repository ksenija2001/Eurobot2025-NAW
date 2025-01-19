import numpy as np
from matplotlib import pyplot as plt
from random import Random
import time

class Point:
    x:float = 0
    y:float = 0
    theta:float = 0
    dtheta:float = 0

def odom_curve(left_inc, right_inc, track, old_pose:Point):
    if left_inc == right_inc:
        return odom_line(left_inc, right_inc, track, old_pose)
    
    ds = (left_inc + right_inc)/2
    dtheta = (right_inc - left_inc)/(2*track)
    #print(f"{dtheta} : {ds}")

    new_theta = old_pose.theta + dtheta
    pose = Point()
    pose.x = old_pose.x + (ds/dtheta) * (np.sin(new_theta) - np.sin(old_pose.theta)) #* np.sin(dtheta)
    pose.y = old_pose.y - (ds/dtheta) * (np.cos(new_theta) - np.cos(old_pose.theta))#(1 - np.cos(dtheta))
    pose.theta = new_theta

    return pose

def odom_line(left_inc, right_inc, track, old_pose:Point):
    ds = (left_inc + right_inc)/2
    dtheta = (right_inc - left_inc)/(2*track)

    pose = Point()
    pose.x = old_pose.x + ds * np.cos(old_pose.theta + dtheta/2)
    pose.y = old_pose.y + ds * np.sin(old_pose.theta + dtheta/2)
    pose.theta = old_pose.theta + dtheta

    return pose

if __name__ == "__main__":
    pose_curve = Point()
    pose_line = Point()

    rand = Random()

    left = 0
    right = 0
    track = 150

    fig, ax = plt.subplots()
    ax.set_xlim((-1000, 1000))
    ax.set_ylim((-1000, 1000))

    ax.plot(0, 0, 'yx')
    start_time = time.time()
    for i in range(1000*100):
        left = rand.randint(-40, 40) * 25*1e-3
        #left = 10 * 25*1e-3
        right = rand.randint(-40, 40) * 25*1e-3

        print(f"{left} : {right}")

        if i%100 == 0:
            ax.plot(pose_line.x, pose_line.y, 'ro')
            ax.plot(pose_curve.x, pose_curve.y, 'bo')

        pose_curve = odom_curve(left, right, track, pose_curve)
        pose_line = odom_line(left, right, track, pose_line)
    

    plt.show()




