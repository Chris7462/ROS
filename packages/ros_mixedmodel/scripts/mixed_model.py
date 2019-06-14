#!/usr/bin/env python

# import ROS, message filters, and numpy & pandas
import rospy
import message_filters
import numpy as np
import pandas as pd
import time

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
class FRM:
    '''Receive vision, track, and vehicle messages and formats the database'''
    TRACK_TOPIC = '/Radar/Tracks/ReducedObjects'
    VEHICLE_TOPIC = '/VehicleState'
    VISION_TOPIC = '/Vision/Lanes'

    def __init__(self):
        # chopping constance
        self.RARE_MAX = 50

        # host msg
        self.host = None

        # database for each source
        self.colnames = ["ID", "world_x", "world_y"]
        self.host_database = pd.DataFrame(columns=self.colnames)
        self.track_database = pd.DataFrame(columns=self.colnames)
        self.vision_database = pd.DataFrame(columns=self.colnames)
        self.vision_buffer = pd.DataFrame(columns=self.colnames)

        # Initialize node
        rospy.init_node('MixedEffectModel', anonymous=True)

        # Initialize vehicle subscriber
        self.vehicle_sub = message_filters.Subscriber(FRM.VEHICLE_TOPIC, VehicleStateMsg)

        # Initialize track subscriber
        self.track_sub = message_filters.Subscriber(FRM.TRACK_TOPIC, F360ObjectsMsg)

        # Initialize vision subscriber
        self.vision_sub = message_filters.Subscriber(FRM.VISION_TOPIC, meLanesMsg)

        # Time Synchronizer
        ats = message_filters.ApproximateTimeSynchronizer([self.vehicle_sub, self.track_sub, self.vision_sub], queue_size=10, slop=0.02)
        #ats = message_filters.ApproximateTimeSynchronizer([self.vehicle_sub, self.vision_sub], queue_size=10, slop=0.05)
        #ats = message_filters.ApproximateTimeSynchronizer([self.vehicle_sub, self.track_sub], queue_size=10, slop=0.02)
        ats.registerCallback(self.callback)

        # Initialize road world model (RWM) publiser
        self.frm_pub = rospy.Publisher('RWM', LaneMarkerMsg, queue_size = 10)

        # publish rate 100 Hz
        self.rate = rospy.Rate(20)

        # running the publisher
        self.frm_publisher()

    def callback(self, vehicle_msg, track_msg, vision_msg):
    #def callback(self, vehicle_msg, vision_msg):
    #def callback(self, vehicle_msg, track_msg):
        #   print [vehicle_msg.header.stamp.to_time(), vehicle_msg.header.stamp.to_nsec()]
        #   print [track_msg.header.stamp.to_time(), track_msg.header.stamp.to_nsec()]
        #   print [vision_msg.header.stamp.to_time(), vision_msg.header.stamp.to_nsec()]
        #   print "got three msgs roughly synced around 0.01 time difference"

        # get host msg
        self.host = vehicle_msg

        # add host to database
        self.add_host_to_database(vehicle_msg)
        # add tracks to database
        self.add_track_to_database(track_msg)
        # add vision to database
        self.add_vision_to_database(vision_msg)

        # chopping database
        self.host_database = chopping(self.host_database, self.host, self.RARE_MAX)
        #rospy.loginfo(self.host_database.shape)
        self.track_database = chopping(self.track_database, self.host, self.RARE_MAX)
        #rospy.loginfo(self.track_database.shape)
        if not self.vision_database.empty:
            self.vision_database = chopping(self.vision_database, self.host, self.RARE_MAX)
            #rospy.loginfo(self.vision_database.shape)

    def add_host_to_database(self, vehicle_msg):
        host_DataFrame = pd.DataFrame([["host", vehicle_msg.world_x, vehicle_msg.world_y]], columns=self.colnames)
        self.host_database = self.host_database.append(host_DataFrame, ignore_index=True)

    def add_track_to_database(self, track_msg):
        # find reduced object ID
        track_info = list(track_msg.tracker_info.reduced_obj_ids)
        obj_id = []
        for i in range(0,len(track_info),1):
            if track_info[i] > 0 :
                obj_id.append(i)

        # store object ID to track_database
        track_DataFrame = pd.DataFrame(columns=self.colnames)
        j = 0
        for i in range(0, len(obj_id),1):
            obj = track_msg.objects[obj_id[i]]
            if not obj.f_oncoming and obj.f_vehicular_track:
                track_DataFrame.loc[j] = [obj.trackID, obj.xposn, obj.yposn]
                j += 1
        # add track to database
        self.track_database = self.track_database.append(track_DataFrame,ignore_index=True)
        #self.track_database = track_DataFrame

    def add_vision_to_database(self, vision_msg):
        sample_size = 4

        # left lane marker
        if vision_msg.left_lane.view_range_end != 0 :
            lc0 = vision_msg.left_lane.position
            lc1 = vision_msg.left_lane.heading_angle
            lc2 = vision_msg.left_lane.curvature
            lc3 = vision_msg.left_lane.curvature_derv
            lvr = vision_msg.left_lane.view_range_end
            vcs_x = np.array([x for x in np.linspace(0,lvr,sample_size)]) # sample 4 points
            vcs_y = lc0+lc1*vcs_x+lc2*vcs_x**2+lc3*vcs_x**3

            g_mat = VCStoWCS(self.host.world_x, self.host.world_y, self.host.heading, vcs_x, vcs_y)
            length = g_mat.shape[0]
            llmid = ['llm']*length

            # create left lanemarker dataframe
            tmp_df = pd.DataFrame({'ID':llmid,'world_x':g_mat[:,0].reshape(1,length).tolist()[0],'world_y':g_mat[:,1].reshape(1,length).tolist()[0]})

            # add left lanemark to database
            self.vision_database = self.vision_database.append(tmp_df.iloc[0:1,],ignore_index=True)
            self.vision_buffer = tmp_df.iloc[1:,]

        # right lane marker
        if vision_msg.right_lane.view_range_end != 0 :
            rc0 = vision_msg.right_lane.position
            rc1 = vision_msg.right_lane.heading_angle
            rc2 = vision_msg.right_lane.curvature
            rc3 = vision_msg.right_lane.curvature_derv
            rvr = vision_msg.right_lane.view_range_end
            vcs_x = np.array([x for x in np.linspace(0,rvr,sample_size)]) # sample 4 points
            vcs_y = rc0+rc1*vcs_x+rc2*vcs_x**2+rc3*vcs_x**3

            g_mat = VCStoWCS(self.host.world_x, self.host.world_y, self.host.heading, vcs_x, vcs_y)
            length = g_mat.shape[0]
            rlmid = ['rlm']*length

            # create right lanemarker dataframe
            tmp_df = pd.DataFrame({'ID':rlmid,'world_x':g_mat[:,0].reshape(1,length).tolist()[0],'world_y':g_mat[:,1].reshape(1,length).tolist()[0]})

            # add right lanemarker to database
            self.vision_database = self.vision_database.append(tmp_df.iloc[0:1,],ignore_index=True)
            self.vision_buffer = self.vision_database.append(tmp_df.iloc[1:,],ignore_index=True)

        # next left lane marker
        if vision_msg.left_lane_next.view_range_end != 0 :
            nlc0 = vision_msg.left_lane_next.position
            nlc1 = vision_msg.left_lane_next.heading_angle
            nlc2 = vision_msg.left_lane_next.curvature
            nlc3 = vision_msg.left_lane_next.curvature_derv
            nlvr = vision_msg.left_lane_next.view_range_end
            vcs_x = np.array([x for x in np.linspace(0,nlvr,sample_size)]) # sample 4 points
            vcs_y = nlc0+nlc1*vcs_x+nlc2*vcs_x**2+nlc3*vcs_x**3

            g_mat = VCStoWCS(self.host.world_x, self.host.world_y, self.host.heading, vcs_x, vcs_y)
            length = g_mat.shape[0]
            nllmid = ['nllm']*length

            # create left lanemarker dataframe
            tmp_df = pd.DataFrame({'ID':nllmid,'world_x':g_mat[:,0].reshape(1,length).tolist()[0],'world_y':g_mat[:,1].reshape(1,length).tolist()[0]})

            # add left lanemark to database
            self.vision_database = self.vision_database.append(tmp_df.iloc[0:1,],ignore_index=True)
            self.vision_buffer = self.vision_database.append(tmp_df.iloc[1:,],ignore_index=True)

        # next right lane marker
        if vision_msg.right_lane_next.view_range_end != 0 :
            nrc0 = vision_msg.right_lane_next.position
            nrc1 = vision_msg.right_lane_next.heading_angle
            nrc2 = vision_msg.right_lane_next.curvature
            nrc3 = vision_msg.right_lane_next.curvature_derv
            nrvr = vision_msg.right_lane_next.view_range_end
            vcs_x = np.array([x for x in np.linspace(0,nrvr,sample_size)]) # sample 4 points
            vcs_y = nrc0+nrc1*vcs_x+nrc2*vcs_x**2+nrc3*vcs_x**3

            g_mat = VCStoWCS(self.host.world_x, self.host.world_y, self.host.heading, vcs_x, vcs_y)
            length = g_mat.shape[0]
            nrlmid = ['nrlm']*length

            # create right lanemarker dataframe
            tmp_df = pd.DataFrame({'ID':nrlmid,'world_x':g_mat[:,0].reshape(1,length).tolist()[0],'world_y':g_mat[:,1].reshape(1,length).tolist()[0]})

            # add right lanemarker to database
            self.vision_database = self.vision_database.append(tmp_df.iloc[0:1,],ignore_index=True)
            self.vision_buffer = self.vision_database.append(tmp_df.iloc[1:,],ignore_index=True)

    def frm_publisher(self):
        start_params = np.array(None)
        while not rospy.is_shutdown():
            if self.vision_database.shape[0] > 5 or self.track_database.shape[0] > 5 :    # need to change to enough_data(self.database)

                # a snapshot for database and the host
                database = pd.concat([self.host_database, self.track_database, self.vision_database, self.vision_buffer], ignore_index=True)
                #database = pd.concat([self.host_database, self.track_database], ignore_index=True)

                # chopping again just in case
                database = chopping(database, self.host, self.RARE_MAX)

                # update VCS_X and VCS_Y of trails
                tmp = WCStoVCS(self.host.world_x, self.host.world_y, self.host.heading, database[['world_x']], database[['world_y']])
                database = database.assign(vcs_x=tmp[:,0], vcs_y=tmp[:,1])
                #rospy.loginfo(database)

                # fit road shape model
                t = time.time()
                road_shape_model = fit_road_shape_model(database)
                t1 = time.time()-t
                rospy.loginfo([t1, road_shape_model.converged])

                # estimate dominant road shape
                est_road_shape = estimate_road_shape(road_shape_model, database)
                host_offset = road_shape_model.random_effects['host'].values[0]

                lane_marker_msg = LaneMarkerMsg()
                lane_marker_msg.header.frame_id = '/vcs'
                lane_marker_msg.header.stamp = rospy.Time()

                road_shape_x = np.array(est_road_shape['vcs_x'])
                road_shape_y = np.array(est_road_shape['vcs_y'])+host_offset

                for i in xrange(len(road_shape_x)):
                    point_msg = PointMsg()
                    point_msg.x = road_shape_x[i]
                    point_msg.y = road_shape_y[i]
                    lane_marker_msg.road_shape.append(point_msg)

                # draw left lane marker
                if 'llm' in road_shape_model.random_effects.keys():
                    llm_offset = road_shape_model.random_effects['llm'].values[0]
                    llm_y = np.array(est_road_shape['vcs_y'])+llm_offset
                    for i in xrange(len(road_shape_x)):
                        point_msg = PointMsg()
                        point_msg.x = road_shape_x[i]
                        point_msg.y = llm_y[i]
                        lane_marker_msg.llm.append(point_msg)

                # draw right lane marker
                if 'rlm' in road_shape_model.random_effects.keys():
                    rlm_offset = road_shape_model.random_effects['rlm'].values[0]
                    rlm_y = np.array(est_road_shape['vcs_y'])+rlm_offset
                    for i in xrange(len(road_shape_x)):
                        point_msg = PointMsg()
                        point_msg.x = road_shape_x[i]
                        point_msg.y = rlm_y[i]
                        lane_marker_msg.rlm.append(point_msg)

                # draw next left lane marker
                if 'nllm' in road_shape_model.random_effects.keys():
                    nllm_offset = road_shape_model.random_effects['nllm'].values[0]
                    nllm_y = np.array(est_road_shape['vcs_y'])+nllm_offset
                    for i in xrange(len(road_shape_x)):
                        point_msg = PointMsg()
                        point_msg.x = road_shape_x[i]
                        point_msg.y = nllm_y[i]
                        lane_marker_msg.nllm.append(point_msg)

                # draw next right lane marker
                if 'nrlm' in road_shape_model.random_effects.keys():
                    nrlm_offset = road_shape_model.random_effects['nrlm'].values[0]
                    nrlm_y = np.array(est_road_shape['vcs_y'])+nrlm_offset
                    for i in xrange(len(road_shape_x)):
                        point_msg = PointMsg()
                        point_msg.x = road_shape_x[i]
                        point_msg.y = nrlm_y[i]
                        lane_marker_msg.nrlm.append(point_msg)

                # publish the lane marker msg
                self.frm_pub.publish(lane_marker_msg)

                self.rate.sleep()

if __name__ == '__main__':
    try:
        frm = FRM()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
