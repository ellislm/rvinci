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


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Vector3.h>
#include <OgreVector3.h>
#include "naviman/rvinci_pose.h"

namespace naviman
{

/*
 *RVinci Pose is a class that allows for a "one-stop-shop" for a lot of common
 *ros position and orientation types. It allows coordinates (for cursor or camera)
 *to be set from geometry_msgs, Ogre, or tf vectors, as well as allowing for the
 *position information to be outputted in those formats as well
*/

 rvinciPose::rvinciPose(float x, float y, float z)
  :x_(x)
  ,y_(y)
  ,z_(z)
    {
    setOffset(getOgreVector());
    }
 rvinciPose::rvinciPose(const rvinciPose& rhs)
  :x_(rhs.x_)
  ,y_(rhs.y_)
  ,z_(rhs.z_)
    {
    }
 rvinciPose::~rvinciPose(){}

  //update is meant to be fed a differential position ogre vector to allow for small, incremental
  //updates to the Pose position.
  void rvinciPose::updatePosition(const Ogre::Vector3& dVector)
    {
    x_ += dVector.x;
    y_ += dVector.y;
    z_ += dVector.z;
    }
  void rvinciPose::updatePosition(const rvinciPose& dVector)
    {
    x_ += dVector.getX();
    y_ += dVector.getY();
    z_ += dVector.getZ();
    }
  void rvinciPose::reset()
    {
    setOgreVector(getOgreVector() + offset_);
    }
  void rvinciPose::setOffset(const Ogre::Vector3& vector){offset_ = vector;}
  void rvinciPose::setOgreVector(const Ogre::Vector3& ovector)
    {
    x_ = ovector.x;
    y_ = ovector.y;
    z_ = ovector.z;
    }
  void rvinciPose::setTFVector(const tf::Vector3& tfvector)
    {
    x_ = tfvector.x();
    y_ = tfvector.y();
    z_ = tfvector.z();
    }
  void rvinciPose::setGMVector(const geometry_msgs::Vector3& gmVector)
    {
    x_ = gmVector.x;
    y_ = gmVector.y;
    z_ = gmVector.z;
    }
  void rvinciPose::setGMPoint(const geometry_msgs::Point& gmPoint)
    {
    x_ = gmPoint.x;
    y_ = gmPoint.y;
    z_ = gmPoint.z;
    }
  void rvinciPose::setXYZ(float x, float y, float z)
    {
    x_ = x, y_ = y, z_ = z;
    }
 
  rvinciPose & rvinciPose::operator = (const rvinciPose& rhs)
  {
    if(this!=&rhs)
      {
      x_ = rhs.x_;
      y_ = rhs.y_;
      z_ = rhs.z_;
      }
  }
  rvinciPose rvinciPose::operator + (const rvinciPose& rhs) const
    {
    return rvinciPose((x_+rhs.x_),(y_+rhs.y_),(z_+rhs.z_));
    }
  rvinciPose rvinciPose::operator - (const rvinciPose& rhs) const
    {
    return rvinciPose((x_-rhs.x_),(y_-rhs.y_),(z_-rhs.z_));
    }
  rvinciPose rvinciPose::operator * (const rvinciPose& rhs) const
    {
    return rvinciPose((x_*rhs.x_),(y_*rhs.y_),(z_*rhs.z_));
    }
  rvinciPose rvinciPose::operator / (const rvinciPose& rhs) const
    {
    return rvinciPose((x_/rhs.x_),(y_/rhs.y_),(z_/rhs.z_));
    }
  rvinciPose rvinciPose::operator += (const rvinciPose& rhs)
    {
    return rvinciPose((x_+=rhs.x_),(y_+=rhs.y_),(z_+=rhs.z_));
    }
  rvinciPose rvinciPose::operator -= (const rvinciPose& rhs)
    {
    return rvinciPose((x_-=rhs.x_),(y_-=rhs.y_),(z_-=rhs.z_));
    }
  rvinciPose rvinciPose::operator *= (const rvinciPose& rhs)
    {
    return rvinciPose((x_*=rhs.x_),(y_*=rhs.y_),(z_*=rhs.z_));
    }
  rvinciPose rvinciPose::operator /= (const rvinciPose& rhs)
    {
    return rvinciPose((x_/=rhs.x_),(y_/=rhs.y_),(z_/=rhs.z_));
    }

}
