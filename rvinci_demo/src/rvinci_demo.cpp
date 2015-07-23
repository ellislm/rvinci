/*
 * Copyright (c) 2011, Willow Garage, Inc.
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


// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");
  ros::NodeHandle n;
  ros::Publisher mpub[3];
  mpub[0]= n.advertise<visualization_msgs::Marker>("visualization_marker0", 1);
  mpub[1] = n.advertise<visualization_msgs::Marker>("visualization_marker1", 1);
  mpub[2] = n.advertise<visualization_msgs::Marker>("visualization_marker2", 1);
  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker[3];
  for(int i = 0; i < 3; ++i)
  {
  int_marker[i].header.frame_id = "/base_link";
  int_marker[i].header.stamp=ros::Time::now();
  int_marker[i].description = "";
  int_marker[i].scale = 0.5;
  }
  int_marker[0].name = "red_move";
  int_marker[1].name = "white_move";
  int_marker[2].name = "blue_move";

  // create a grey box marker
  visualization_msgs::Marker marker[6];
  for (int i = 0; i<6; ++i)
  {
  marker[i].type = visualization_msgs::Marker::CUBE;
  marker[i].header.frame_id = "/base_link";
  marker[i].scale.x = 0.25;
  marker[i].scale.y = 0.25;
  marker[i].scale.z = 0.25;
  marker[i].color.a = 1.0;
  marker[i].pose.position.y = 0.0;
  marker[i].pose.position.z = 0.0;
  marker[i].pose.position.x = 0.0;
  }

  marker[0].color.r = 1.0;
  marker[0].color.g = 0.0;
  marker[0].color.b = 0.0;
  marker[0].scale.z = 0.75;
  marker[0].pose.position.x = -0.5;
  marker[0].pose.position.z = 0.25;

  marker[1].color.r = 1.0;
  marker[1].color.g = 1.0;
  marker[1].color.b = 1.0;
  marker[1].scale.x = 0.75;

  marker[2].color.r = 0.0;
  marker[2].color.g = 0.0;
  marker[2].color.b = 1.0;
  marker[2].scale.z = 0.75;
  marker[2].pose.position.x = 0.5;
  marker[2].pose.position.z = 0.25;

  marker[3].color.r = 0.7;
  marker[3].color.g = 0.05;
  marker[3].color.b = 0.05;
  marker[3].pose.position.y = 1.25;
  marker[3].pose.position.x = 0;

  marker[4].color.r = 0.65;
  marker[4].color.g = 0.65;
  marker[4].color.b = 0.65;
  marker[4].scale.z = 0.75;
  marker[4].scale.x = 0.75;
 marker[4].pose.position.y = 1;
marker[4].pose.position.x = 0.5;

  marker[5].color.r = 0.05;
  marker[5].color.g = 0.05;
  marker[5].color.b = 0.8;
  marker[5].pose.position.y = 1;
 marker[5].pose.position.x = -0.45;


  // create a non-interactive control which contains the box
for(int i = 0; i<3;++i)
{
  visualization_msgs::InteractiveMarkerControl control[3];
  control[i].always_visible = true;
 control[i].markers.push_back(marker[i+3]);
  int_marker[i].controls.push_back( control[i] );
  int_marker[i].controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  server.insert(int_marker[i]);
  server.setCallback(int_marker[i].name, &processFeedback);
}
   server.applyChanges();
  while(ros::ok())
  {
  for(int i = 0; i<3; ++i) mpub[i].publish(marker[i]);
  ros::spinOnce();
  }
  }
