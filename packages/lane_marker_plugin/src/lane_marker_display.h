/*! @file lane_marker_display.h
*   @brief Adds lane marker message as a display type in ROS
*/
/*  *** Aptiv Electronics & Safety ***

Copyright 2018 by Aptiv PLC. All Rights Reserved.
This file contains Aptiv Proprietary information.
It may not be reproduced or distributed without permission.
*/


#ifndef LaneMarker_DISPLAY_H
#define LaneMarker_DISPLAY_H

#include <deque>
#include <queue>
#include <vector>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include "rviz/display_context.h"
#include <lane_marker_messages/LaneMarker.h>
#include <rviz/message_filter_display.h>


namespace Ogre
{
class SceneNode;
class Vector3;
class Quaternion;
}

namespace rviz
{
class ColorProperty;
class FloatProperty; 
class DisplayContext;
class PointCloud;
class BillboardLine;
class Shape;
}

namespace lane_marker_plugin
{
// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// TrailDisplay will show a PointCloud showing the trail points
// of the object trails.  The base of the trails will be at
// the frame listed in the header of the TrailDatabase message.
//
// The TrailDisplay class itself implements editable parameters, and Display subclass machinery.
class LaneMarkerDisplay : public rviz::MessageFilterDisplay<lane_marker_messages::LaneMarker>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  LaneMarkerDisplay();
  ~LaneMarkerDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties. 
private Q_SLOTS:
  void updateColorAndAlpha();

// Function to handle an incoming ROS message.
private:
  void processMessage(const lane_marker_messages::LaneMarker::ConstPtr& msg);

  rviz::BillboardLine* lines_;

  //User-editable property variables.
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
};
} //end namespace lane_marker_plugin
#endif

