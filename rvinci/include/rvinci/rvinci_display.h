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
Q_OBJECT
public:
  rvinciDisplay();
  virtual ~rvinciDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

  // Overrides from Ogre::RenderTargetListener
  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

protected:

  virtual void onEnable();
  virtual void onDisable();
  void cameraUpdate();
protected Q_SLOTS:
  virtual void cameraReset();
  virtual void pubsubSetup();
private:
  void cameraSetup();
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);
  void publishCursorUpdate(int grab[2]);
  int getaGrip(bool, int);
  bool camera_mode_, clutch_mode_;
  bool prev_grab_[2];

  Ogre::Camera* camera_;
  Ogre::SceneNode *camera_node_;
  Ogre::SceneNode *target_node_;
  Ogre::Viewport *viewport_[2];
  Ogre::RenderWindow *window_;

  Ogre::Vector3 initial_cvect_;
  Ogre::Vector3 camera_offset_;
  Ogre::Vector3 camera_pos_;
  Ogre::Quaternion camera_ori_;
  Ogre::Vector3 input_pos_[2];
  Ogre::Vector3 input_change_[2];

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_camera_;
  ros::Publisher publisher_rhcursor_;
  ros::Publisher publisher_lhcursor_;
  ros::Publisher publisher_left_hand_;

  rviz::VectorProperty *prop_cam_focus_;
  rviz::QuaternionProperty *property_camrot_;
  rviz::BoolProperty *prop_manual_coords_;
  rviz::VectorProperty *prop_camera_posit_;
  rviz::VectorProperty *prop_input_scalar_;
  rviz::RosTopicProperty *prop_ros_topic_;
  rviz::BoolProperty *prop_cam_reset_;
  rviz::RenderWidget *render_widget_;

  tf::Transform camera_tf_;
  tf::TransformBroadcaster br_;

  geometry_msgs::Pose cursor_[2];
 /*
#ifndef Q_MOC_RUN
  tf::TransformBroadcaster tf_pub_;
  boost::shared_ptr<Oculus> oculus_;
#endif*/
};

} // namespace rvinci

#endif

