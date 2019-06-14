#!/usr/bin/env python

import rospy
import message_filters

# import messages
from f360_tracker_messages.msg import F360Objects as F360ObjectsMsg
from vehicle_state_messages.msg import VehicleState as VehicleStateMsg
from mobileye_melanes_messages.msg import meLanes as meLanesMsg

rospy.init_node('sync_test')

def gotmsg(vehicle,track,vision):
    print [vehicle.header.stamp.to_time(), vehicle.header.stamp.to_nsec()]
    print [track.header.stamp.to_time(), track.header.stamp.to_nsec()]
    print [vision.header.stamp.to_time(), vision.header.stamp.to_nsec()]
    print "got three msgs roughly synced around 0.02 time difference"

vehicle_sub = message_filters.Subscriber('/VehicleState', VehicleStateMsg)
track_sub = message_filters.Subscriber('/Radar/Tracks/ReducedObjects', F360ObjectsMsg)
vision_sub = message_filters.Subscriber('/Vision/Lanes', meLanesMsg)

ats = message_filters.ApproximateTimeSynchronizer([vehicle_sub,track_sub,vision_sub], queue_size=10, slop=0.01)
ats.registerCallback(gotmsg)

rospy.spin()
