/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#ifndef RVINCI_DISPLAY_H
#define RVINCI_DISPLAY_H

#include "rviz/display.h"

#include <QObject>
#include <ros/ros.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgrePrerequisites.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <rvinci_input_msg/rvinci_input.h>

namespace Ogre
{
class SceneNode;
class RenderWindow;
class Camera;
class Viewport;
}

namespace rviz
{
class BoolProperty;
class RenderWidget;
class VectorProperty;
class QuaternionProperty;
class RosTopicProperty;
}

namespace rvinci
{
class rvinciDisplay: public rviz::Display, public Ogre::RenderTargetListener
{
//! RVinci display plugin for RViz.
/*! The RVinci display class is a plugin for RViz which is designed to allow
 * a da Vinci surgical console to navigate the RViz environment and manipulate
 * virtual objects within the world. It spawns a seperate window with stereo display
 * whose cameras can be controlled with the console. It also provides outputs for
 * the interaction_cursor_3D to spawn two 3D cursors. 
 */
Q_OBJECT
public:
   //! A constructor
   /*!The rviz/Qt Render Widget is created here, and the Ogre
   * rendering window is attached to it. The Ogre camera node
   * is spawned and the ROS subscriber and publisher setup member is called.
   */
  rvinciDisplay();
  //!Destructor
  virtual ~rvinciDisplay();

//  virtual void reset();

  //!Override from Ogre::RenderTargetListener
  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

  //!Override from Ogre::RenderTargetListener
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

protected:
  //!Called after onInitialize.
  /*!Called after onInitialize or if display plugin is enabled
   * after being disabled. Calls camera setup member if cameras
   * are not initialized and makes external render window visible.
   */
  virtual void onEnable();
  //!Called when plugin is disabled (by deselecting the check box).
  virtual void onDisable();
  //!Contains primary logic for camera control.
  /*!Camera position is either manually entered, or calculated by position
   * of the da Vinci grips when the camera pedal is activated. A vector is
   * calculated between the right and left grips. The translation of the midpoint
   * of this vector is added to the camera node position, and the change in orientation
   * of this vector is added to the orientation of the camera node.
   */
  void cameraUpdate();
  //!Called after constructor
  virtual void onInitialize();
  //!Override from rviz display class.
  virtual void update( float wall_dt, float ros_dt );
protected Q_SLOTS:
  //!Resets or intializes camera and 3D cursor positions.
  virtual void cameraReset();
  //!Sets up ROS subscribers and publishers
  virtual void pubsubSetup();
  //!Toggle for DVRK Gravity Compensation state
  virtual void gravityCompensation();
private:
  //!Creates viewports and cameras.
  void cameraSetup();
  //!Called when input message received.
  /*!Contains primary input logic. Records input position and calculates change in
   * input position. Updates cursor position then sends data to camera control and cursor publisher.
   */
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);
  //!Publishes cursor position and grip state to interaction cursor 3D display type.
  void publishCursorUpdate(int grab[2]);
  //!Logic for grip state, used in interaction cursor 3D display type.
  int getaGrip(bool, int);
  bool camera_mode_, clutch_mode_;
  bool prev_grab_[2];

  Ogre::Camera* camera_[2];
  Ogre::SceneNode *camera_node_;
  Ogre::SceneNode *target_node_;
  Ogre::Viewport *viewport_[2];
  Ogre::RenderWindow *window_;

  Ogre::Vector3 initial_cvect_;
  Ogre::Vector3 camera_ipd_;
  Ogre::Vector3 camera_offset_;
  Ogre::Vector3 cursor_offset_[2];
  Ogre::Vector3 camera_pos_;
  Ogre::Quaternion camera_ori_;
  Ogre::Vector3 input_pos_[2];
  Ogre::Vector3 input_change_[2];

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_input_;
  ros::Publisher publisher_rhcursor_;
  ros::Publisher publisher_lhcursor_;
  ros::Publisher pub_robot_state_[2];

  rviz::VectorProperty *prop_cam_focus_;
  rviz::QuaternionProperty *property_camrot_;
  rviz::BoolProperty *prop_manual_coords_;
  rviz::VectorProperty *prop_camera_posit_;
  rviz::VectorProperty *prop_input_scalar_;
  rviz::RosTopicProperty *prop_ros_topic_;
  rviz::BoolProperty *prop_gravity_comp_;
  rviz::BoolProperty *prop_cam_reset_;

  rviz::RenderWidget *render_widget_;

  geometry_msgs::Pose cursor_[2];

};

} // namespace rvinci

#endif

