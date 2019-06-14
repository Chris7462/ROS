#!/usr/bin/env python

import rospy
import pandas as pd
from f360_tracker_messages.msg import F360Objects as F360ObjectsMsg
from vehicle_state_messages.msg import VehicleState as VehicleStateMsg
from mobileye_melanes_messages.msg import meLanes as meLanesMsg

class FRMDatabase:
    '''Receive vision, track, and vehicle messages and formats the database'''
    VISION_TOPIC = '/Vision/Lanes'
    TRACK_TOPIC = '/Radar/Tracks/ReducedObjects'
    VEHICLE_TOPIC = '/VehicleState'

    def __init__(self);
        self.database = pd.DataFrame(columns=["ID", "global_x", "global_y"])


    

col_names = ["ID", "world_x", "world_y"]
database = pd.DataFrame(columns=col_names)  # an empty database

def add_track_to_database(database, tracks_msgs):
    track_info = list(tracks_msgs.tracker_info.reduced_obj_ids)
    obj_id = []
    for i in range(0,len(track_info),1):
        if track_info[i] > 0:
            obj_id.append(i)

    col_names = ["ID", "world_x", "world_y"]
    track_database = pd.DataFrame(columns=col_names)
    j = 0

    for i in range(0,len(obj_id),1):
        obj = tracks_msgs.objects[obj_id[i]]
        if not obj.f_oncoming and obj.f_vehicular_track:
            track_database.loc[j] = [obj.trackID, obj.xposn, obj.yposn]
            j += 1

    database = database.append(track_database,ignore_index=True)

    return(database)


def track_callback(msg):
    tmp = add_track_to_database(database, msg)
    rospy.loginfo(tmp[["ID"]].values[-1])
    rospy.loginfo(tmp.shape)
    #rospy.loginfo(msg.tracker_info.tracker_index)
    #rospy.loginfo(msg.tracker_info.reduced_obj_ids)

    #def vehicle_callback(msg):
    #    rospy.loginfo(msg.world_x)

    #def vision_callback(msg):
    #    rospy.loginfo(msg.update_index)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('MixedModel', anonymous=True)

    # subscribe F360 Object message
    rospy.Subscriber('/Radar/Tracks/ReducedObjects', F360ObjectsMsg, track_callback)

    # subscribe Vehicle State message
    #rospy.Subscriber('/VehicleState', VehicleStateMsg, vehicle_callback)

    # subscribe Vision message
    #rospy.Subscriber('/Vision/Lanes', meLanesMsg, vision_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
