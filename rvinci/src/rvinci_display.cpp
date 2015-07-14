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
#include <rvinci_input_msg/rvinci_input.h>

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
  ytoz_ = Ogre::Quaternion(sqrt(0.5),-1*sqrt(0.5),0,0);
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
  xyz_Scalar_ = new rviz::VectorProperty("X,Y,Z Scalars",Ogre::Vector3(3,3,3),
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
 camera_pose_.setXYZ(0,0, 0);
 camera_node_->setOrientation(1,0,0,0);
 camera_node_->setPosition(camera_pose_.getOgreVector());
// camera_node_->setFixedYawAxis(true);
 camera_node_->attachObject(camera_);
 camera_->setNearClipDistance(0.01f);
 camera_->setFarClipDistance(10000.0f);
 camera_->setPosition(0,-3,1.5);
 camera_->lookAt(camera_node_->getPosition());

 for(int i = 0; i<2; ++i)
 {
   viewport_[i] = window_->addViewport(camera_,i,0.5f*i,0.0f,0.5f,1.0f);//,0,0.5f,0,0.5f,1.0f);
   viewport_[i]->setBackgroundColour(bg_color);
 }
 //General cursor offset for startup
 cursor_[0].setXYZ(-0.5,0,0), cursor_[1].setXYZ(0.5,0.0,0);
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
  subscriber_camera_ = nh_.subscribe<rvinci_input_msg::rvinci_input>("/davinci_msg",10, boost::bind(&rvinciDisplay::inputCallback,this,_1));
  publisher_rhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_right/update",10);
  publisher_lhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_left/update",10);
  publisher_left_hand_ = nh_.advertise<geometry_msgs::Pose>("dvrk_mtml/set_position_cartesian",10);
}
void rvinciDisplay::inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input)
{
  camera_mode_ = r_input->camera;
  clutch_mode_ = r_input->clutch;
      //Ogre::Quaternion ytoz_ = Ogre::Quaternion(pi()/2, Ogre::Vector3(0,0,1))*Ogre::Quaternion(pi()/2,Ogre::Vector3(1,0,0));
  if(!clutch_mode_)
  {
    rvinciPose right_old_ = input_pose_[1];
    for (int i = 0; i<2; ++i)
    {
      rvinciPose old_input = input_pose_[i];
      input_pose_[i].setGMPose(r_input->gripper[i].pose);
      input_change_[i].setOgreVector(camera_->getRealOrientation()*(input_pose_[i].getOgreVector() - old_input.getOgreVector()));
      input_change_[i].setOgreQuaternion(input_pose_[i].getOgreQuaternion()
                                          *old_input.getOgreQuaternion().Inverse());
     grab_[i] = r_input->gripper[i].grab;
    }
    if (!camera_mode_)
    {
      for (int i = 0; i<2; ++i)
      {
        cursor_[i].updatePosition(input_change_[i].getOgreVector() * xyz_Scalar_->getVector());
        cursor_[i].setOgreQuaternion(cursor_[i].getOgreQuaternion()*input_change_[i].getOgreQuaternion());
      }
        publishCursorUpdate();
        initial_cvect_ = (input_pose_[0].getOgreVector() - input_pose_[1].getOgreVector() - Ogre::Vector3(0.3,0,0));
        initial_cvect_.normalise();
    }
    else
      {
        updateCamera();

      }
  }
  else//to avoid an erroneously large input_update_ following clutched movement
  {
    for(int i = 0; i<2; ++i)
    {
      input_pose_[i].setGMPose(r_input->gripper[i].pose);
    }
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
  rhcursor.pose.pose.orientation = cursor_[1].getGMQuaternion();
  rhcursor.button_state = grab_[1];
  lhcursor.pose.header.frame_id = frame.toStdString();
  lhcursor.pose.header.stamp = ros::Time::now();
  lhcursor.pose.pose.orientation = cursor_[0].getGMQuaternion();
  lhcursor.pose.pose.position = cursor_[0].getGMPoint();
  lhcursor.button_state = grab_[0];

  publisher_rhcursor_.publish(rhcursor);
  publisher_lhcursor_.publish(lhcursor);
}
void rvinciDisplay::updateCamera()
{
  
  if(use_manual_coords_->getBool())
   {
     camera_pose_.setOgreVector(camera_offset_->getVector()); 
     camera_->setPosition(camera_pose_.getOgreVector());
  property_camrot_->setQuaternion(camera_->getRealOrientation());
  camera_->lookAt(ytoz_*property_camfocus_->getVector());
  //   camera_->setFixedYawAxis(true, camera_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
    }
  if(!use_manual_coords_->getBool() && camera_mode_)
    {
     // Ogre::Quaternion ytoz_(sqrt(0.5),-1*sqrt(0.5),0,0);
      Ogre::Vector3 newvect =(input_pose_[0].getOgreVector() - input_pose_[1].getOgreVector() - Ogre::Vector3(0.4,0,0));
      newvect.normalise();
      Ogre::Quaternion camrot  = initial_cvect_.getRotationTo(newvect);
      property_camrot_->setQuaternion(camera_node_->getOrientation()*camrot);
      camera_pose_.setOgreVector(camera_pose_.getOgreVector() - ((input_change_[_RIGHT].getOgreVector()+ input_change_[0].getOgreVector())*Ogre::Vector3(2,2,2)));
      camera_node_->setOrientation(camera_node_->getOrientation()*camrot);
      camera_node_->setPosition(camera_pose_.getOgreVector());
      //camera_->lookAt(camera_node_->getPosition());
    camera_tf_.setOrigin(target_pose_.getTFVector());
    camera_tf_.setRotation(tf::Quaternion(0.0,0.0,0.0,1));
    initial_cvect_ = newvect;
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
