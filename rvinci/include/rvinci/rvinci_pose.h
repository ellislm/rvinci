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

#ifndef RVINCI_POSE_H
#define RVINCI_POSE_H


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_datatypes.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgrePrerequisites.h>
namespace rvinci
{

/*
 *RVinci Pose is a class that allows for a "one-stop-shop" for a lot of common
 *ros position and orientation types. It allows coordinates (for cursor or camera)
 *to be set from geometry_msgs, Ogre, or tf vectors, as well as allowing for the
 *position information to be outputted in those formats as well
*/

class rvinciPose
{
public:
  rvinciPose(float x=0, float y=0, float z=0, float ox=0, float oy=0,float oz=0,float ow=1);
  rvinciPose(const rvinciPose & rhs);
  virtual ~rvinciPose();
  Ogre::Vector3 getOgreVector() const {return Ogre::Vector3(x_,y_,z_);}
  Ogre::Quaternion getOgreQuaternion() const {return Ogre::Quaternion(ow_,ox_,oy_,oz_);} 
  tf::Vector3 getTFVector() const {return tf::Vector3(x_,y_,z_);}
  geometry_msgs::Vector3 getGMVector() const {geometry_msgs::Vector3 vector;
                                              vector.x = x_, vector.y = y_, vector.z = z_;
                                              return vector;}
  geometry_msgs::Point getGMPoint() const {geometry_msgs::Point point;
                                              point.x = x_, point.y = y_, point.z = z_;
                                              return point;}

 geometry_msgs::Quaternion getGMQuaternion() const {geometry_msgs::Quaternion rot;
                                              rot.x = ox_, rot.y = oy_, rot.z = oz_, rot.w = ow_ ;
                                              return rot;}
  //initialized if offset has been configured
// bool isInitialized() const {if(offset_) return true;
 //                             else {return false};}
 //update is meant to be fed a differential position ogre vector to allow for small, incremental
  //updates to the Pose position.
  void updatePosition(const Ogre::Vector3& dVector);
  void updatePosition(const rvinciPose& dVector);
  void reset();
  void setOffset(const Ogre::Vector3&);
  void setOgreVector(const Ogre::Vector3&);
  void setOgreQuaternion(const Ogre::Quaternion&);
  void setTFVector(const tf::Vector3&);
  void setGMVector(const geometry_msgs::Vector3&);
  void setGMPoint(const geometry_msgs::Point&);
  void setGMPose(const geometry_msgs::Pose&);
  void setGMQuaternion(const geometry_msgs::Quaternion&);
   rvinciPose & operator = (const rvinciPose&);
  rvinciPose operator + (const rvinciPose&) const;
  rvinciPose operator - (const rvinciPose&) const;
  rvinciPose operator * (const rvinciPose&) const;
  rvinciPose operator / (const rvinciPose&) const;
  rvinciPose operator += (const rvinciPose&);
  rvinciPose operator -= (const rvinciPose&);
  rvinciPose operator *= (const rvinciPose&);
  rvinciPose operator /= (const rvinciPose&);


  float getX() const {return x_;}
  float getY() const {return y_;}
  float getZ() const {return z_;}
  float getOX() const {return ox_;}
  float getOY() const {return oy_;}
  float getOZ() const {return oz_;}
  float getOW() const {return ow_;}
  
  void setQuaternion(float ox, float oy, float oz, float ow);
  void setXYZ(float x, float y, float z);
  void setXYZ(const rvinciPose&); 

  private:

  Ogre::Vector3 offset_;
  float x_, ox_;
  float y_, oy_;
  float z_, oz_;
  float ow_;
  };
}
#endif
