/*! @file lane_marker_display.cpp
*   @brief Adds lane marker message as a display type in ROS
*/
/*  *** Aptiv Electronics & Safety ***

Copyright 2018 by Aptiv PLC. All Rights Reserved.
This file contains Aptiv Proprietary information.
It may not be reproduced or distributed without permission.
*/


#include "lane_marker_display.h"
#include "rviz/default_plugin/marker_display.h"
#include "rviz/selection/selection_manager.h"
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>
#include <math.h>


namespace lane_marker_plugin
{
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
LaneMarkerDisplay::LaneMarkerDisplay()
: lines_(0)
{

  color_property_ = new rviz::ColorProperty( "Color", QColor( 240, 210, 0 ),
                                             "Color to draw the lane markers.",
                                             this, SLOT( updateColorAndAlpha() ));
  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));
}

LaneMarkerDisplay::~LaneMarkerDisplay()
{
  delete lines_;
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void LaneMarkerDisplay::onInitialize()
{
  MFDClass::onInitialize();
  
  if(!lines_)
  {
    lines_ = new rviz::BillboardLine(context_->getSceneManager(), scene_node_);
  }
}

// Clear the trail points.
void LaneMarkerDisplay::reset()
{
  MFDClass::reset();
  lines_->clear();
}

// Set the current color and alpha values.
void LaneMarkerDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
}

// This is our callback to handle an incoming message.
void LaneMarkerDisplay::processMessage(const lane_marker_messages::LaneMarker::ConstPtr& msg)
{
  
  if(!lines_)
  {
    lines_ = new rviz::BillboardLine(context_->getSceneManager(), scene_node_);
  }
  


  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this TrailDatabase message.  If
  // it fails, we can't do anything else so we return.
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }
  

  Ogre::Vector3 scale(0.1,0.1,0.1);

  lines_->setScale(scale);
  lines_->setLineWidth(0.25);


  scene_node_->createChildSceneNode()->setPosition(position);
  scene_node_->createChildSceneNode()->setOrientation(orientation);

  lines_->clear();

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  lines_->setMaxPointsPerLine(msg->road_shape.size());
  //lines_->setNumLines(3);
  lines_->setNumLines(4);
  
  //Load data from the received message.
  //for(int i = 0;i<msg->road_shape.size();i++)
  //{
  //    Ogre::Vector3 v(msg->road_shape[i].x, msg->road_shape[i].y, 0 );
  //    Ogre::ColourValue line_color(color.r,color.g,color.b);
  //    lines_->addPoint(v,line_color);    
  //}  

  //lines_->newLine();
  for(int i = 0;i<msg->llm.size();i++)
  {
      Ogre::Vector3 v(msg->llm[i].x, msg->llm[i].y, 0 );
      Ogre::ColourValue line_color(color.r,color.g,color.b);
      lines_->addPoint(v,line_color);    
  }  

  lines_->newLine();
  for(int i = 0;i<msg->rlm.size();i++)
  {
      Ogre::Vector3 v(msg->rlm[i].x, msg->rlm[i].y, 0 );
      Ogre::ColourValue line_color(color.r,color.g,color.b);
      lines_->addPoint(v,line_color);    
  }  

  lines_->newLine();
  for(int i = 0;i<msg->nllm.size();i++)
  {
      Ogre::Vector3 v(msg->nllm[i].x, msg->nllm[i].y, 0 );
      Ogre::ColourValue line_color(color.r,color.g,color.b);
      lines_->addPoint(v,line_color);    
  }  

  lines_->newLine();
  for(int i = 0;i<msg->nrlm.size();i++)
  {
      Ogre::Vector3 v(msg->nrlm[i].x, msg->nrlm[i].y, 0 );
      Ogre::ColourValue line_color(color.r,color.g,color.b);
      lines_->addPoint(v,line_color);    
  }  
}
} // end namespace trail_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lane_marker_plugin::LaneMarkerDisplay, rviz::Display)
