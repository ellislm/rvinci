/* * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <iostream>

#include <QWidget>
#include <QDesktopWidget>
#include <QApplication>

//#include <OVR.h>

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>

#include <ros/package.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>
#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include <rvinci_input_msg/rvinci_input.h>

#include "rvinci/rvinci_display.h"

#define _RIGHT 1
#define _LEFT 0

namespace rvinci
{
rvinciDisplay::rvinciDisplay()
: render_widget_(0)
, camera_node_(0)
, window_(0)
, camera_(0)
, camera_offset_(0.0,-3.0,1.5)
{
  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);

  prop_ros_topic_ = new rviz::RosTopicProperty("Input Topic","/davinci_msg"
                                               ,ros::message_traits::datatype<rvinci_input_msg::rvinci_input>(),
                                               "Subscription topic (published by input controller node)"
                                               ,this,SLOT ( pubsubSetup()));
  prop_input_scalar_ = new rviz::VectorProperty("Input Scalar",Ogre::Vector3(5,5,5),
                                                "Scalar for X, Y, and Z of controller inputi motion",this);
  prop_cam_reset_ = new rviz::BoolProperty("Camera Reset",false,
                                           "Reset camera and cursor position", this, SLOT (cameraReset()));
  prop_manual_coords_ = new rviz::BoolProperty("Use typed coordinates",false,
                                               "Camera movement controlled by typed coordinates",this);
  prop_cam_focus_ = new rviz::VectorProperty("Camera Focus",Ogre::Vector3(0,0,0),
                                             "Focus Point of Camera",this);
  prop_camera_posit_ = new rviz::VectorProperty("Camera Position",camera_offset_,
                                                 "Position of scene node to world base frame",this);
  property_camrot_ = new rviz::QuaternionProperty("Camera Orientation",Ogre::Quaternion(0,0,0,1),
                                                  "Orientation of the camera",this);
}
rvinciDisplay::~rvinciDisplay()
{
  for(int i = 0; i<2; ++i)
  {
    if (viewport_[i])
   {
      window_->removeViewport(0);
      viewport_[i] = 0;
    }
  }
  if (camera_)
   {
      camera_->getParentSceneNode()->detachObject(camera_);
      scene_manager_->destroyCamera(camera_);
      camera_ = 0;
   }
  if (camera_node_)
  {
    camera_node_->getParentSceneNode()->removeChild(camera_node_);
    scene_manager_->destroySceneNode(camera_node_);
    camera_node_ = 0;
  }
  window_ = 0;
  delete render_widget_;
  delete prop_manual_coords_;
  delete prop_cam_focus_;
  delete prop_camera_posit_;
  delete prop_input_scalar_;
}

//Overrides from rviz Display
void rvinciDisplay::onInitialize()
{
  render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("RVinci");
  render_widget_->resize(2800,1050);
  render_widget_->show();
  render_widget_->setWindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint);
  window_ = render_widget_->getRenderWindow();
  window_->setVisible(false);
  window_->setAutoUpdated(false);
  window_->addListener(this);
  camera_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  target_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  pubsubSetup();
}
void rvinciDisplay::update(float wall_dt, float ros_dt)
{
  cameraUpdate();
  window_ = render_widget_->getRenderWindow();
  window_->update(false);
}
void rvinciDisplay::reset(){}
void rvinciDisplay::pubsubSetup()
{
  std::string subtopic = prop_ros_topic_->getStdString();
  subscriber_camera_ = nh_.subscribe<rvinci_input_msg::rvinci_input>(subtopic, 10, boost::bind(&rvinciDisplay::inputCallback,this,_1));
  publisher_rhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_right/update",10);
  publisher_lhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_left/update",10);
  publisher_left_hand_ = nh_.advertise<geometry_msgs::Pose>("dvrk_mtml/set_position_cartesian",10);
}
void rvinciDisplay::inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input)
{
  Ogre::Quaternion orshift(sqrt(0.5),sqrt(0.5),0,0);
  orshift=orshift* Ogre::Quaternion(sqrt(0.5),0,sqrt(0.5),0);
  Ogre::Vector3 cursoffset(0.3,0,0);
  cursoffset*=prop_input_scalar_->getVector();//Ogre::Quaternion ytoz_ = Ogre::Quaternion(pi()/2, Ogre::Vector3(0,0,1))*Ogre::Quaternion(pi()/2,Ogre::Vector3(1,0,0));
  Ogre::Quaternion inori[2];

  camera_mode_ = r_input->camera;
  clutch_mode_ = r_input->clutch;

  if(!clutch_mode_)
  {
    for (int i = 0; i<2; ++i)
    {
      Ogre::Vector3 old_input = input_pos_[i];
      geometry_msgs::Pose pose = r_input->gripper[i].pose;

      input_pos_[i] = Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z);
      input_pos_[i]*=prop_input_scalar_->getVector();
      inori[i] = Ogre::Quaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
      inori[i]= inori[i]*orshift;

      input_change_[i] = camera_->getRealOrientation()*(input_pos_[i] - old_input);
    }

    if (!camera_mode_)
    {
      geometry_msgs::Pose curspose;

      int grab[2];
      for (int i = 0; i<2; ++i)
      {
        cursor_[i].position.x += input_change_[i].x;
        cursor_[i].position.y += input_change_[i].y;
        cursor_[i].position.z += input_change_[i].z;
        cursor_[i].orientation.x = inori[i].x;
        cursor_[i].orientation.y = inori[i].y;
        cursor_[i].orientation.z = inori[i].z;
        cursor_[i].orientation.w = inori[i].w;
        grab[i] = getaGrip(r_input->gripper[i].grab, i);
//        cursori[i].setOgreQuaternion(input_pose_[i].getOgreQuaternion());//cursor_[i].getOgreQuaternion()*input_change_[i].getOgreQuaternion());
      }
        publishCursorUpdate(grab);
        initial_cvect_ = (input_pos_[_LEFT] - input_pos_[_RIGHT] - cursoffset);
        initial_cvect_.normalise();
    }
    else
      {
        cameraUpdate();
      }
  }
  else//to avoid an erroneously large input_update_ following clutched movement
  {
    for(int i = 0; i<2; ++i)
    {
      geometry_msgs::Pose pose = r_input->gripper[i].pose;
     // input_pos_[i].setGMPose(r_input->gripper[i].pose);
      //input_pos_[i].setOgreVector(input_pose_[i].getOgreVector()*prop_input_scalar_->getVector());
      input_pos_[i] = Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z);
      input_pos_[i]*= prop_input_scalar_->getVector();
      initial_cvect_ = (input_pos_[_LEFT] - input_pos_[_RIGHT] - cursoffset);
      initial_cvect_.normalise();
    }
  }
}
void rvinciDisplay::publishCursorUpdate(int grab[2])
{
  //fixed frame is a parent member from RViz Display, pointing to selected world frame in rviz;
  std::string frame = context_->getFixedFrame().toStdString();
  interaction_cursor_msgs::InteractionCursorUpdate lhcursor;
  interaction_cursor_msgs::InteractionCursorUpdate rhcursor;

  rhcursor.pose.header.frame_id = frame;
  rhcursor.pose.header.stamp = ros::Time::now();
  rhcursor.pose.pose = cursor_[_RIGHT];
//  rhcursor.pose.pose.orientation = cursori[_RIGHT];
  rhcursor.button_state = grab[_RIGHT];
  lhcursor.pose.header.frame_id = frame;
  lhcursor.pose.header.stamp = ros::Time::now();
//  lhcursor.pose.pose.orientation = cursori[_LEFT];
  lhcursor.pose.pose = cursor_[_LEFT];
  lhcursor.button_state = grab[_LEFT];

  publisher_rhcursor_.publish(rhcursor);
  publisher_lhcursor_.publish(lhcursor);
}
int rvinciDisplay::getaGrip(bool grab, int i)
{
  if(grab && !prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 2;//Grab object
    }
  if(grab && prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 1;//hold object
    }
  if(!grab && prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 3;//Release object
    }
  if(!grab && !prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 0;//none
   } 
}
void rvinciDisplay::cameraSetup()
{
 Ogre::ColourValue bg_color = context_->getViewManager()->getRenderPanel()->getViewport()->getBackgroundColour();
 window_ = render_widget_->getRenderWindow();
 camera_ = scene_manager_->createCamera("Camera");
 camera_node_->attachObject(camera_);
 cameraReset();
 for(int i = 0; i<2; ++i)
 {
   viewport_[i] = window_->addViewport(camera_,i,0.5f*i,0.0f,0.5f,1.0f);//,0,0.5f,0,0.5f,1.0f);
   viewport_[i]->setBackgroundColour(bg_color);
 }
}
void rvinciDisplay::cameraReset()
{
 camera_pos_= Ogre::Vector3(0.0f,0.0f,0.0f);
 camera_node_->setOrientation(1,0,0,0);
 camera_node_->setPosition(camera_pos_);
 camera_->setNearClipDistance(0.01f);
 camera_->setFarClipDistance(10000.0f);
 camera_->setPosition(camera_offset_);
 camera_->lookAt(camera_node_->getPosition());

 for (int i = 0; i<2; ++i)
 {
 cursor_[i].position.x = -0.5 + i;
 cursor_[i].position.y = 0.0;
 cursor_[i].position.z = 0.0;
 }

 prop_cam_reset_->setValue(QVariant(false));
}
void rvinciDisplay::cameraUpdate()
{
  if(prop_manual_coords_->getBool())
   {
     camera_pos_ = Ogre::Vector3(prop_camera_posit_->getVector());
     camera_->setPosition(camera_pos_);
     property_camrot_->setQuaternion(camera_->getRealOrientation());
     camera_->lookAt(prop_cam_focus_->getVector());
    }
  if(!prop_manual_coords_->getBool() && camera_mode_)
    {
      Ogre::Vector3 cursoffset(0.3,0,0);
      cursoffset*=prop_input_scalar_->getVector();

      Ogre::Vector3 newvect = input_pos_[_LEFT] - input_pos_[_RIGHT] - cursoffset;
      newvect.normalise();
      Ogre::Quaternion camrot  = initial_cvect_.getRotationTo(newvect);
      camera_pos_ = Ogre::Vector3(camera_pos_ - ((input_change_[_RIGHT] + input_change_[_LEFT])));
      camera_node_->setOrientation(camera_node_->getOrientation()*camrot.Inverse());
      camera_node_->setPosition(camera_pos_);
      initial_cvect_ = newvect;

      property_camrot_->setQuaternion(camera_node_->getOrientation()*camrot);
      prop_camera_posit_->setVector(camera_pos_ + property_camrot_->getQuaternion()*camera_->getPosition());
      prop_cam_focus_->setVector(camera_node_->getPosition());
    }
}
//Overrides from OgreTargetListener
void rvinciDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
cameraUpdate();
}
void rvinciDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  window_ = render_widget_->getRenderWindow();
  window_->swapBuffers();
}
void rvinciDisplay::onEnable()
{
  if(!camera_)
  {
  cameraSetup();
  }
  render_widget_->setVisible(true);
}
void rvinciDisplay::onDisable()
{
  render_widget_ ->setVisible(false);
}

}//namespace rvinci
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rvinci::rvinciDisplay, rviz::Display )
