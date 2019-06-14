#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd
from f360_tracker_messages.msg import F360Objects as F360ObjectsMsg
from vehicle_state_messages.msg import VehicleState as VehicleStateMsg
from mobileye_melanes_messages.msg import meLanes as meLanesMsg
from common_func import VCStoWCS

class FRMDatabase:
    '''Receive vision, track, and vehicle messages and formats the database'''
    TRACK_TOPIC = '/Radar/Tracks/ReducedObjects'
    VEHICLE_TOPIC = '/VehicleState'
    VISION_TOPIC = '/Vision/Lanes'

    def __init__(self):
        self.veh_time = 0
        self.colnames = ["ID", "world_x", "world_y"]
        self.database = pd.DataFrame(columns=self.colnames)
        self.host_buffer = None
        self.track_buffer = None
        self.vision_buffer = None
        self.vision_sample_step = 20

        # Initialize node
        rospy.init_node('FRMDatabase', anonymous=True)

        # Initialize track subscriber
        self.track_sub = rospy.Subscriber(FRMDatabase.TRACK_TOPIC, F360ObjectsMsg, self.track_callback)

        # Initialize vehicle subscriber
        self.vehicle_sub = rospy.Subscriber(FRMDatabase.VEHICLE_TOPIC, VehicleStateMsg, self.vehicle_callback)

        # Initialize vision subscriber
        #self.vision_sub = rospy.Subscriber(FRMDatabase.VISION_TOPIC, meLanesMsg, self.vision_callback)

    def add_track_to_database(self):    # okay!
        # find reduced object ID
        if self.track_buffer:
            track_info = list(self.track_buffer.tracker_info.reduced_obj_ids)
            obj_id = []
            for i in range(0,len(track_info),1):
                if track_info[i] > 0 :
                    obj_id.append(i)

            # store object ID to track_database
            track_database = pd.DataFrame(columns=self.colnames)
            j = 0
            for i in range(0, len(obj_id),1):
                obj = self.track_buffer.objects[obj_id[i]]
                if not obj.f_oncoming and obj.f_vehicular_track:
                    track_database.loc[j] = [obj.trackID, obj.xposn, obj.yposn]
                    j += 1
            # add track to database
            self.database = self.database.append(track_database,ignore_index=True)

    def add_host_to_database(self):     # okay!
        hid = "host"
        world_x = self.host_buffer.world_x
        world_y = self.host_buffer.world_y
        # add host to database
        self.database.loc[len(self.database)] = [hid, world_x, world_y]

    def add_vision_to_database(self):
        #if np.abs(np.array((self.host_buffer.header.stamp.to_time()-self.vision_buffer.header.stamp.to_time()))) < 0.02 :
        rospy.loginfo(np.array(self.host_buffer.header.stamp.to_time()-self.vision_buffer.header.stamp.to_time()))
        rospy.loginfo(np.abs(np.array(self.host_buffer.header.stamp.to_time()-self.vision_buffer.header.stamp.to_time())) < 0.02)

        # left lane marker
        if self.vision_buffer.left_lane.view_range_end != 0 :
            lc0 = self.vision_buffer.left_lane.position
            lc1 = self.vision_buffer.left_lane.heading_angle
            lc2 = self.vision_buffer.left_lane.curvature
            lc3 = self.vision_buffer.left_lane.curvature_derv
            lvr = self.vision_buffer.left_lane.view_range_end
            vcs_x = np.array([x for x in range(0,int(lvr),self.vision_sample_step)])
            vcs_y = lc0+lc1*vcs_x+lc2*vcs_x**2+lc3*vcs_x**3

            #g_mat = self.VCStoWCS(self.host_buffer.world_x, self.host_buffer.world_y, self.host_buffer.heading, vcs_x, vcs_y)
            g_mat = VCStoWCS(self.host_buffer.world_x, self.host_buffer.world_y, self.host_buffer.heading, vcs_x, vcs_y)
            length = g_mat.shape[0]
            llmid = ['llm']*length

            # create left lanemarker dataframe
            tmp_df = pd.DataFrame({'ID':llmid,'world_x':g_mat[:,0].reshape(1,length).tolist()[0],'world_y':g_mat[:,1].reshape(1,length).tolist()[0]})

            # add left lanemark to database
            self.database = self.database.append(tmp_df,ignore_index=True)

        # right lane marker
        if self.vision_buffer.right_lane.view_range_end != 0 :
            rc0 = self.vision_buffer.right_lane.position
            rc1 = self.vision_buffer.right_lane.heading_angle
            rc2 = self.vision_buffer.right_lane.curvature
            rc3 = self.vision_buffer.right_lane.curvature_derv
            rvr = self.vision_buffer.right_lane.view_range_end
            vcs_x = np.array([x for x in range(0,int(rvr),self.vision_sample_step)])
            vcs_y = rc0+rc1*vcs_x+rc2*vcs_x**2+rc3*vcs_x**3

            #g_mat = self.VCStoWCS(self.host_buffer.world_x, self.host_buffer.world_y, self.host_buffer.heading, vcs_x, vcs_y)
            g_mat = VCStoWCS(self.host_buffer.world_x, self.host_buffer.world_y, self.host_buffer.heading, vcs_x, vcs_y)
            length = g_mat.shape[0]
            rlmid = ['rlm']*length

            # create right lanemarker dataframe
            tmp_df = pd.DataFrame({'ID':rlmid,'world_x':g_mat[:,0].reshape(1,length).tolist()[0],'world_y':g_mat[:,1].reshape(1,length).tolist()[0]})

            # add right lanemarker to database
            self.database = self.database.append(tmp_df,ignore_index=True)

    def vehicle_callback(self,msg):
        self.host_buffer = msg

        #rospy.loginfo(self.host_buffer.header.stamp-self.vision_buffer.header.stamp)
        #rospy.loginfo(self.vision_buffer.header.stamp)

        self.add_host_to_database()
        #self.add_track_to_database()
        #self.add_vision_to_database()
        #rospy.loginfo(self.database)

    def track_callback(self,msg):
        self.track_buffer = msg

    def vision_callback(self,msg):
        self.vision_buffer = msg

if __name__ == '__main__':
    try:
        frm_database = FRMDatabase()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
