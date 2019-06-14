#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd

# import messages
from f360_tracker_messages.msg import F360Objects as F360ObjectsMsg
from vehicle_state_messages.msg import VehicleState as VehicleStateMsg
from mobileye_melanes_messages.msg import meLanes as meLanesMsg

# import custom functions
from trail_func import *
from common_func import *

from lane_marker_messages.msg import LaneMarker as LaneMarkerMsg
from lane_marker_messages.msg import Point as PointMsg

#class FRMDatabase(threading.Thread):
class FRMDatabase:
    '''Receive vision, track, and vehicle messages and formats the database'''
    TRACK_TOPIC = '/Radar/Tracks/ReducedObjects'
    VEHICLE_TOPIC = '/VehicleState'
    VISION_TOPIC = '/Vision/Lanes'

    def __init__(self):
        # chopping constance
        self.RARE_MAX = 50

        self.colnames = ["ID", "world_x", "world_y", "vcs_x", "vcs_y"]
        # database for each source
        self.host_database = pd.DataFrame(columns=self.colnames)
        self.track_database = pd.DataFrame(columns=self.colnames)
        self.vision_database = pd.DataFrame(columns=self.colnames)

        #self.database = pd.DataFrame(columns=self.colnames)
        self.host = None

        # Initialize node
        rospy.init_node('FRMDatabase', anonymous=True)

        # Initialize vehicle subscriber
        self.vehicle_sub = rospy.Subscriber(FRMDatabase.VEHICLE_TOPIC, VehicleStateMsg, self.vehicle_callback)

        # Initialize track subscriber
        self.track_sub = rospy.Subscriber(FRMDatabase.TRACK_TOPIC, F360ObjectsMsg, self.track_callback)

        # Initialize vision subscriber
        #self.vision_sub = rospy.Subscriber(FRMDatabase.VISION_TOPIC, meLanesMsg, self.vision_callback)

        # Initialize road world model (RWM) publiser
        self.frm_pub = rospy.Publisher('RWM', LaneMarkerMsg, queue_size = 10)

        # publish rate 10 Hz
        self.rate = rospy.Rate(10)

        # running the publisher
        self.frm_publisher()

    def vehicle_callback(self,msg):
        rospy.loginfo(['vehicle state ', msg.header.stamp])
        # save current host!
        self.host = msg
        # append the current host information to the host database
        #host_DataFrame = pd.DataFrame([["host", self.host.world_x, self.host.world_y, 0, 0]], columns=self.colnames)
        #self.host_database = self.host_database.append(host_DataFrame, ignore_index=True)
        #self.host_database = host_DataFrame

        # chopping trails
        #self.host_database = chopping(self.host_database, self.host, self.RARE_MAX)
        #rospy.loginfo(["host", self.host_database.shape])

    def track_callback(self,msg):
        rospy.loginfo(['track: ', msg.header.stamp])
        # find reduced object ID
        track_info = list(msg.tracker_info.reduced_obj_ids)
        obj_id = []
        for i in range(0,len(track_info),1):
            if track_info[i] > 0 :
                obj_id.append(i)

        # store object ID to track_database
        track_DataFrame = pd.DataFrame(columns=self.colnames)
        j = 0
        for i in range(0, len(obj_id),1):
            obj = msg.objects[obj_id[i]]
            if not obj.f_oncoming and obj.f_vehicular_track:
                track_DataFrame.loc[j] = [obj.trackID, obj.xposn, obj.yposn, obj.vcs_xposn, obj.vcs_yposn]
                j += 1
        # add track to database
        #self.track_database = self.track_database.append(track_DataFrame,ignore_index=True)
        self.track_database = track_DataFrame
        #rospy.loginfo(self.track_database)

        # chopping trails
        #self.track_database = chopping(self.track_database, self.host, self.RARE_MAX)
        #rospy.loginfo(self.track_database)

    def frm_publisher(self):
        #count = 0
        while not rospy.is_shutdown(): 
            #count += 1
            if self.track_database.shape[0] > 1 or self.host_database.shape[0] > 1 :    # need to change to enough_data(self.database)

                # a snapshot for database and the host
                #database = pd.concat([self.host_database, self.track_database], ignore_index=True)
                host_msg = self.host
                database = self.track_database
                #rospy.loginfo(database)
                #rospy.loginfo(["merged", database.shape])

                # chopping again just in case
                #database = chopping(database, self.host, self.RARE_MAX)

                # update VCS_X and VCS_Y of trails
                #tmp = WCStoVCS(host_msg.world_x, host_msg.world_y, host_msg.heading, database[['world_x']], database[['world_y']])
                #database = database.assign(my_vcs_x=tmp[:,0], my_vcs_y=tmp[:,1])

                #rospy.loginfo(database)

                # fit road shape model
                road_shape_model = fit_road_shape_model(database)

                # estimate dominant road shape
                est_road_shape = estimate_road_shape(road_shape_model, database)
                #rospy.loginfo(est_road_shape['vcs_x'])

                lane_marker_msg = LaneMarkerMsg()
                lane_marker_msg.header.frame_id = '/vcs'
                lane_marker_msg.header.stamp = rospy.Time()

                road_shape_x = np.array(est_road_shape['vcs_x'])
                road_shape_y = np.array(est_road_shape['vcs_y'])

                for i in xrange(len(road_shape_x)):
                    point_msg = PointMsg()
                    point_msg.x = road_shape_x[i]
                    point_msg.y = road_shape_y[i]

                    lane_marker_msg.road_shape.append(point_msg)

                self.frm_pub.publish(lane_marker_msg)
                #rospy.loginfo(lane_marker_msg.road_shape[0].x)

                self.rate.sleep()

if __name__ == '__main__':
    try:
        frm_database = FRMDatabase()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
