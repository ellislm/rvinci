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

#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h> 
#include <interaction_cursor_msgs/InteractionCursorUpdate.h> 
#include <rvinci_cursor_msg/rvinci_cursor.h>

#include "rvinci/rvinci_display.h" 
#define _x 0 
#define _y 1 
#define _z 2 
#define _RIGHT 1
namespace rvinci
{
rvinciDisplay::rvinciDisplay()
: render_widget_(0)
, camera_node_(0)
, window_(0)
{
  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);
//  connect( QApplication::desktop(), SIGNAL( screenCountChanged ( int ) ), this, SLOT( onScreenCountChanged(int)) );
  camera_=0;
}
rvinciDisplay::~rvinciDisplay()
{
  for(int i = 0; i<2; ++i)
   if (viewport_[i])
   {
      window_->removeViewport(0);
      viewport_[i] = 0;
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
  delete use_manual_coords_;
  delete property_camfocus_;
  delete camera_offset_;
  delete xyz_Scalar_;
}

//Overrides from rviz Display
void rvinciDisplay::onInitialize()
{
  use_manual_coords_ = new rviz::BoolProperty("Typed Coords",false,
          "Typed camera coordinates override hydra control",this);
//  camera_Position_ = new rviz::VectorProperty("Position",Ogre::Vector3(-0.032f,0,0),
  //                            "Position of camera to world base frame",this);
  property_camfocus_ = new rviz::VectorProperty("Focus",Ogre::Vector3(0,0,0),
                                              "Focus Point of Camera",this);
  camera_offset_ = new rviz::VectorProperty("SN Position",Ogre::Vector3(0,0,0),
                          "Position of scene node to world base frame",this);
  xyz_Scalar_ = new rviz::VectorProperty("X,Y,Z Scalars",Ogre::Vector3(1,1,1),
                          "Scalars for X, Y, and Z of controller motion input",this);
  property_targposit_= new rviz::VectorProperty("Target Node Position",Ogre::Vector3(0,0,0),
                          "Position of scene node to world base frame",this);
  property_camrot_ = new rviz::QuaternionProperty("camrot",Ogre::Quaternion(0,0,0,1),
                          "Position of scene node to world base frame",this);
  render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("RVinci");
  render_widget_->resize(1680,1050);
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
void rvinciDisplay::cameraSetup()
  {

 Ogre::ColourValue bg_color = context_->getViewManager()->getRenderPanel()->getViewport()->getBackgroundColour();
 window_ = render_widget_->getRenderWindow();
 camera_ = scene_manager_->createCamera("Camera");
 camera_node_->setOrientation(Ogre::Quaternion(0,0,0,1));
 camera_node_->setPosition(camera_offset_->getVector());
// camera_node_->setAutoTracking(true, target_node_);
 camera_node_->setFixedYawAxis(true);
 camera_node_->attachObject(camera_);
 camera_->setNearClipDistance(0.01f);
 camera_->setFarClipDistance(10000.0f);
 camera_->setPosition(0,-3,1.5);
 for(int i = 0; i<2; ++i)
 {
   viewport_[i] = window_->addViewport(camera_,0,0.0f,0.0f,1.0f,1.0f);//,0,0.5f,0,0.5f,1.0f);
   viewport_[i]->setBackgroundColour(bg_color);
 }
 //initializing camera based on typed values
 camera_node_->setPosition(0,0,0);
 camera_->lookAt(camera_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
 camera_->setFixedYawAxis(true, camera_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
}
void rvinciDisplay::update(float wall_dt, float ros_dt)
{
  updateCamera();
  window_ = render_widget_->getRenderWindow();
  window_->update(false);
}
void rvinciDisplay::reset(){}

void rvinciDisplay::pubsubSetup()
{
  subscriber_camera_ = nh_.subscribe<rvinci_input_msg::rvinci_input>("davinci_msg",10, &rvinciDisplay::inputCallback,this);
  publisher_rhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_right/update",10);
  publisher_lhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_left/update",10);
}
void rvinciDisplay::inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input)
{
for (int i = 0; i<2; ++i)
{
  rvinciPose old_input = input_pose_[i];
  input_pose_[i].setGMPose(r_input.gripper[i].pose);
  input_change_[i].setOgreVector((input_pose_[i].getOgreVector() - old_input.getOgreVector())
                   *xyz_Scalar_->getVector);
  input_change_[i]->setOgreQuaternion(input_change_[i]->getOgreQuaternion()
                                      *old_input->getOgreQuaternion().inverse());
}

right_trigger_ = hydra_sub->paddles[1].trigger;
right_bumper_ = hydra_sub->paddles[1].buttons[0];
left_bumper_ = hydra_sub->paddles[0].buttons[0];

if (!right_bumper_)
{
for (int i = 0; i<2; ++i){cursor_[i].updatePosition(input_change_[i]);}
publishCursorUpdate();
}
}
void rvinciDisplay::publishCursorUpdate()
{
  QString frame = fixed_frame_;
  interaction_cursor_msgs::InteractionCursorUpdate lhcursor;
  interaction_cursor_msgs::InteractionCursorUpdate rhcursor;
  //fixed frame is a parent member from RViz Display, pointing to selected world frame in rviz;
  rhcursor.pose.header.frame_id = frame.toStdString();
  rhcursor.pose.header.stamp = ros::Time::now();
  rhcursor.pose.pose.position = cursor_[1].getGMPoint();

  lhcursor.pose.header.frame_id = frame.toStdString();
  lhcursor.pose.header.stamp = ros::Time::now();
  lhcursor.pose.pose.position = cursor_[0].getGMPoint();

  publisher_rhcursor_.publish(rhcursor);
  publisher_lhcursor_.publish(lhcursor);
}
void rvinciDisplay::updateCamera()
{
 sn_Pose_.setOffset(camera_offset_->getVector());

  if(use_manual_coords_->getBool())
   {
  camera_node_->setPosition(camera_offset_->getVector());
  target_node_->setPosition(property_targposit_->getVector());
  property_camrot_->setQuaternion(camera_->getRealOrientation());
  camera_->lookAt(property_camfocus_->getVector());
  //   camera_->setFixedYawAxis(true, camera_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
    }
  if(!use_manual_coords_->getBool() && right_bumper_)
    {
      Ogre::Quaternion shift90(sqrt(0.5),-1*sqrt(0.5),0,0);
      shift90= shift90*(Ogre::Quaternion(sqrt(0.5),0,0,sqrt(0.5))); 
      Ogre::Quaternion camrot = camera_->getRealOrientation();
      camrot = camrot*shift90;
      property_camrot_->setQuaternion(camrot);
      target_pose_.updatePosition(camrot*input_change_[_RIGHT].getOgreVector());
      camera_->lookAt(property_targposit_->getVector());
      property_targposit_->setVector(target_pose_.getOgreVector());
//    target_pose_.updatePosition(input_change_[_RIGHT]);
//    target_node_->setPosition(target_pose_.getOgreVector());
//    property_camfocus_->setVector(target_pose_.getOgreVector());
//    for (int i = 0; i<2; ++i)
//      {
//      camera_[i]->setFixedYawAxis(true, camera_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
//      }
    camera_tf_.setOrigin(target_pose_.getTFVector());
    camera_tf_.setRotation(tf::Quaternion(0.0,0.0,0.0,1));
    if(left_bumper_)
     {
       sn_Pose_.updatePosition(camrot*input_change_[_RIGHT].getOgreVector());
       camera_node_->setPosition(sn_Pose_.getOgreVector());
    }
  }
br_.sendTransform(tf::StampedTransform(camera_tf_, ros::Time::now(), "base_link","/camera_frame"));
}
//Overrides from OgreTargetListener
void rvinciDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
updateCamera();
}
void rvinciDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  window_ = render_widget_->getRenderWindow();
  window_->swapBuffers();
}
void rvinciDisplay::onEnable()
{
 // if(!camera_)

  cameraSetup();

  render_widget_->setVisible(true);
}
void rvinciDisplay::onDisable()
{
render_widget_ ->setVisible(false);
}

//Q_SLOTS will be populated here as necessary.

/* PLACEHOLDERS
rviz::BoolProperty* boolproperty_
rviz::FloatProperty*
rviz::TfFrameProperty*
rviz::VectorProperty*
*/
}//namespace rvinci
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rvinci::rvinciDisplay, rviz::Display )
