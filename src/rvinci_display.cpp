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
  n_cameras_[0] = 0, n_cameras_[1] = 0;
}
rvinciDisplay::~rvinciDisplay()
{
  for (int i = 0; i < 2; ++i)
  {
   if (n_viewports[i])
   {
      window_->removeViewport(i);
      n_viewports[i] = 0;
    }
    if (n_cameras_[i])
   {
      n_cameras_[i]->getParentSceneNode()->detachObject(n_cameras_[i]);
      scene_manager_->destroyCamera(n_cameras_[i]);
      n_cameras_[i] = 0;
   }
  }
  if (camera_node_)
  {
    camera_node_->getParentSceneNode()->removeChild(camera_node_);
    scene_manager_->destroySceneNode(camera_node_);
    camera_node_ = 0;
  }
//  scene_manager_ = 0;
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
  camera_offset_ = new rviz::VectorProperty("SN Position",Ogre::Vector3(0,-5,2),
                          "Position of scene node to world base frame",this);
  xyz_Scalar_ = new rviz::VectorProperty("X,Y,Z Scalars",Ogre::Vector3(1,1,1),
                          "Scalars for X, Y, and Z of controller motion input",this);
  render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("RVinci");
  render_widget_->resize(1280,480);
  render_widget_->show();
  render_widget_->setWindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint);
  window_ = render_widget_->getRenderWindow();
  // window_->setVisible(false);
   window_->setAutoUpdated(false);
   //window_->addListener(this); 
   camera_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
   target_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
 // pubsubSetup();
}
void rvinciDisplay::cameraSetup()
{
  // Ogre::ColourValue bg_color = context_->getViewManager()->getRenderPanel()->getViewport()->getBackgroundColour();
  // window_ = render_widget_->getRenderWindow();
  // n_cameras_[0] = scene_manager_->createCamera("Camera_Left");
  // n_cameras_[1] = scene_manager_->createCamera("Camera_Right");
  // camera_node_->setOrientation(Ogre::Quaternion(0,0,0,1));
  // camera_node_->setPosition(camera_offset_->getVector());
  // camera_node_->setAutoTracking(true, target_node_);
  // camera_node_->setFixedYawAxis(true);
  // for(int i = 0; i<2; ++i)
  // {
  // camera_node_->attachObject(n_cameras_[i]);
  // n_cameras_[i]->setNearClipDistance(0.01f);
  // n_cameras_[i]->setFarClipDistance(10000.0f);
  // n_cameras_[i]->setPosition((i*2 -1)*0.064f*0.5f,0,0);
  // n_viewports[i] = window_->addViewport(n_cameras_[i],i,0.5f*i,0,0.5f,1.0f);
  // n_viewports[i]->setBackgroundColour(bg_color);
  // }
}
void rvinciDisplay::update(float wall_dt, float ros_dt)
{
  // updateCamera();
  // window_ = render_widget_->getRenderWindow();
  // window_->update(false);
}
void rvinciDisplay::reset(){}

void rvinciDisplay::pubsubSetup()
{
//   subscriber_camera_ = nh_.subscribe<razer_hydra::Hydra>("hydra_calib",10, &rvinciDisplay::inputCallback,this);
//   publisher_rhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_right/update",10);
//   publisher_lhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_left/update",10);
// }
// void rvinciDisplay::inputCallback(const razer_hydra::Hydra::ConstPtr& hydra_sub)
// {
//   rvinciPose xyzscale;
//   xyzscale.setOgreVector(xyz_Scalar_->getVector());
// for (int i = 0; i<2; ++i)
// {
//   rvinciPose old_input = input_pose_[i];
//   input_pose_[i].setGMVector(hydra_sub->paddles[i].transform.translation);
//   input_change_[i].setOgreVector(input_pose_[i].getOgreVector() - old_input.getOgreVector());
//   input_change_[i]*=xyzscale;
// }

// right_trigger_ = hydra_sub->paddles[1].trigger;
// right_bumper_ = hydra_sub->paddles[1].buttons[0];
// left_bumper_ = hydra_sub->paddles[0].buttons[0];

// if (!right_bumper_)
// {
// for (int i = 0; i<2; ++i){cursor_[i].updatePosition(input_change_[i]);}
//publishCursorUpdate();
//}
}
void rvinciDisplay::publishCursorUpdate()
{
  // interaction_cursor_msgs::InteractionCursorUpdate lhcursor;
  // interaction_cursor_msgs::InteractionCursorUpdate rhcursor;

  // rhcursor.pose.header.frame_id = "/camera_frame";
  // rhcursor.pose.header.stamp = ros::Time::now();
  // rhcursor.pose.pose.position = cursor_[1].getGMPoint();

  // lhcursor.pose.header.frame_id = "/camera_frame";
  // lhcursor.pose.header.stamp = ros::Time::now();
  // lhcursor.pose.pose.position = cursor_[0].getGMPoint();

  // publisher_rhcursor_.publish(rhcursor);
  // publisher_lhcursor_.publish(lhcursor);
}
void rvinciDisplay::updateCamera()
{
  // sn_Pose_.setOffset(camera_offset_->getVector());
  // if(use_manual_coords_->getBool())
  //   {
  //     camera_node_->setPosition(camera_offset_->getVector());
  //   for (int i = 0; i<2; ++i)
  //     {
  //     n_cameras_[i]->lookAt(property_camfocus_->getVector());
  //     n_cameras_[i]->setFixedYawAxis(true, camera_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
  //     }
  //   }
  // if(!use_manual_coords_->getBool() && right_bumper_)
  //   {
  //   Ogre::Quaternion quart = n_cameras_[0]->getRealOrientation();
  //    tf::Transformer tnodetf(tf::Quaternion(quart.x,quart.y,quart.z,quart.w),
  //                                         camera_Pose_.getTFVector());
  //    rvinciPose rpose;
  //    rpose.setTFVector(tnodetf(input_change_[_RIGHT].getTFVector()));
  //    camera_Pose_.updatePosition(rpose);
  //    camera_Pose_.updatePosition(input_change_[_RIGHT]);
  //    target_node_->setPosition(camera_Pose_.getOgreVector());
  //    property_camfocus_->setVector(camera_Pose_.getOgreVector());
  //    for (int i = 0; i<2; ++i)
  //      {
  //      n_cameras_[i]->setFixedYawAxis(true, camera_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
  //      }
  //    camera_tf_.setOrigin(camera_Pose_.getTFVector());
  //    camera_tf_.setRotation(tf::Quaternion(0.0,0.0,0.0,1));
  //    if(left_bumper_)
  //     {
  //        sn_Pose_.updatePosition(input_change_[_RIGHT]);
  //       camera_node_->setPosition(sn_Pose_.getOgreVector());
  //    }
  //  }
 // br_.sendTransform(tf::StampedTransform(camera_tf_, ros::Time::now(), "base_link","/camera_frame"));
}
//Overrides from OgreTargetListener
void rvinciDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
updateCamera();
}
void rvinciDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  // window_ = render_widget_->getRenderWindow();
  // window_->swapBuffers();
}
void rvinciDisplay::onEnable()
{
  // if(!n_cameras_[0])
  // {
  // cameraSetup();
  // }
  // render_widget_->setVisible(true);
}
void rvinciDisplay::onDisable()
{
//render_widget_ ->setVisible(false);
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
