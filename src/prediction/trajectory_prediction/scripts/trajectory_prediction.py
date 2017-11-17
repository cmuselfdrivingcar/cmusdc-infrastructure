#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from pedestrian_tracking.msg import PedestrianPose
from pedestrian_tracking.msg import PedestrianPoseList
from Queue import Queue, heapq, deque
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped

import numpy as np
import matplotlib.pyplot as plt

QUEUE_SIZE = 12
prediction_length = 12
polyfit_degree = 2

def callback(data):
    currentFrame = data.frameID
    for pose in data.poses:
        pedID = pose.pedID
        if pedsQueue.has_key(pedID):
            queue = pedsQueue[pedID]
            if len(queue) == QUEUE_SIZE:
                queue.pop(0)
            queue.append((pose.frameID, pose.x, pose.y))
            # queue.sort()
        else:
            queue = [(pose.frameID, pose.x, pose.y)]
            pedsQueue[pedID] = queue

    for pedID in pedsQueue:
        queue = pedsQueue[pedID]
        observation_length = len(queue)
        if currentFrame - queue[-1][0] >= 30:
            del pedsQueue[pedID]
            continue
        if observation_length < 3:
            continue
        # frames = [data for data in range(observation_length+prediction_length,0,-1)]
        # xs = [data[1] for data in queue]
        # ys = [data[2] for data in queue]

        # x_pred = np.zeros(prediction_length)
        # y_pred = np.zeros(prediction_length)

        # p_x = np.polyfit(frames[:], xs[:], polyfit_degree)
        # p_y = np.polyfit(frames[:], ys[:], polyfit_degree)

        # p_x = np.poly1d(p_x)
        # p_y = np.poly1d(p_y)

        # startingFrame = frames[0]
        # for i in range(prediction_length):
        #     x_pred[i] = p_x(startingFrame+i+1)
        #     y_pred[i] = p_y(startingFrame+i+1)


        # validate model
        # total_length = len(queue)
        # if total_length < 12:
        #     continue
        # observation_length = total_length-prediction_length

        frames = [frame for frame in range(observation_length)]
        xs = [data[1] for data in queue]
        ys = [data[2] for data in queue]

        x_pred = np.zeros(prediction_length)
        y_pred = np.zeros(prediction_length)

        p_x = np.polyfit(frames[:observation_length], xs[:observation_length], polyfit_degree)
        p_y = np.polyfit(frames[:observation_length], ys[:observation_length], polyfit_degree)

        p_x = np.poly1d(p_x)
        p_y = np.poly1d(p_y)

        # shape_error = (prediction_length, 2)
        # error = np.zeros(shape_error)
        for i in range(prediction_length):
            x_pred[i] = p_x(observation_length+i)
            y_pred[i] = p_y(observation_length+i)
            # error[i][0] = x_pred[i] - xs[i + observation_length]
            # error[i][1] = y_pred[i] - ys[i + observation_length]

        points_pred = list()

        pose_list_pred = list()
        my_path_pred = Path()
        my_path_pred.header.frame_id = 'velodyne'

        pose_list_true = list()
        my_path_true = Path()
        my_path_true.header.frame_id = 'velodyne'

        for i in range(len(x_pred)):
            pose = PoseStamped()
            loc = Pose()
            loc.position.x = x_pred[i]
            loc.position.y = y_pred[i]
            pose.pose = loc

            point = Point()
            point.x = x_pred[i]
            point.y = y_pred[i]
            point.y = 1

            # pose.header.frame_id = '/odom'
            # pose.header.stamp = rospy.Time.now()
            pose_list_pred.append(pose)
            my_path_pred.poses.append(pose)
            points_pred.append(point)

        for i in range(len(xs)):
            pose = PoseStamped()
            loc = Pose()
            loc.position.x = xs[i]
            loc.position.y = ys[i]
            pose.pose = loc

            # pose.header.frame_id = '/odom'
            # pose.header.stamp = rospy.Time.now()
            pose_list_true.append(pose)
            my_path_true.poses.append(pose)

        print '-----------------------------------'
        print y_pred
        print ys[observation_length:]
        # print error

        path_pub.publish(my_path_true)
        predicted_path_pub.publish(my_path_pred)
        # predicted_point_pub.publish(points_pred)
        break
        # print y_pred

    # print xs
    # print ys
    # frames = [data[0] for data in queue]
    # print frames



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/track_points_msg", PedestrianPoseList, callback)
    global path_pub, predicted_path_pub, predicted_point_pub

    path_pub = rospy.Publisher("/path",Path,queue_size=100)
    predicted_path_pub = rospy.Publisher("/predicted_path",Path,queue_size=100)
    predicted_point_pub = rospy.Publisher("/predicted_points",Point,queue_size=100)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    global pedsQueue
    pedsQueue = {}
    listener()
