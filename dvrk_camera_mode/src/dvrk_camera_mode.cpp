#include <ros/ros.h>
#include <rvinci_input_msg/rvinci_input.h>
#include <geometry_msgs/Wrench.h>
#include <cmath>
#include <tf/transform_datatypes.h>
class dvrk_wrench
{
public:
  dvrk_wrench();
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);
  double vector_magnitude_;
  tf::Vector3 ori_magnitude_[2];
  ros::Rate* r_;
private:
  ros::NodeHandle n;
  ros::Subscriber rvinci_sub;
  ros::Publisher dvrk_pub[2];
  double strength_[2]; //current and previous
  double torqmag_[2];
  double torqmago_[2];
  void publishWrench(const tf::Vector3, const tf::Vector3[], double, tf::Transform[]);
  tf::Vector3 getVector(const rvinci_input_msg::rvinci_input);
  bool checkLimits(const rvinci_input_msg::rvinci_input);
};

dvrk_wrench::dvrk_wrench()
{
  rvinci_sub = n.subscribe<rvinci_input_msg::rvinci_input>("/rvinci_input_update",10,&dvrk_wrench::inputCallback,this);
  dvrk_pub[0] = n.advertise<geometry_msgs::Wrench>("/dvrk_mtml/set_wrench",10);
  dvrk_pub[1] = n.advertise<geometry_msgs::Wrench>("/dvrk_mtmr/set_wrench",10);
  ori_magnitude_[0] = ori_magnitude_[1] = tf::Vector3(0,0,0);
}
void dvrk_wrench::inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input)
{
  tf::Vector3 curvect = getVector(*r_input);
 // if (checkLimits(*r_input))
 // {
    tf::Transform tfT[2];
    tf::Vector3 orivect[2];
    for (int i = 0; i<2; ++i)
    {
      tf::Quaternion tfq;
      tf::quaternionMsgToTF(r_input->gripper[i].pose.orientation,tfq); 
      tfT[i] = tf::Transform(tfq.inverse());//*tf::Quaternion(-M_PI/2, 0,0));
      orivect[i] = tf::Transform(tfq)*tf::Vector3(1,1,1);
   //   orivect[i].normalize();
    }
  if(!r_input->camera && curvect.length() >  0.12)
  {
  vector_magnitude_ = curvect.length();
  for(int i = 0; i<2; ++i)
  {
    ori_magnitude_[i] = orivect[i];
  }
  publishWrench(curvect,ori_magnitude_,vector_magnitude_, tfT);
  }
  else
  {
    double curmag = curvect.length();
    std::cout << ori_magnitude_[0].x() << "," << ori_magnitude_[0].y() << "," <<ori_magnitude_[0].z() << std::endl;
    curvect.normalize();
    publishWrench(curvect,orivect, curmag, tfT);
  }
 // }
}
tf::Vector3 dvrk_wrench::getVector(const rvinci_input_msg::rvinci_input rinput)
{
//  geometry_msgs::Vector3 vector;
//  vector.x = rinput.gripper[0].pose.position.x - rinput.gripper[1].pose.position.x;
//  vector.y = rinput.gripper[0].pose.position.y - rinput.gripper[1].pose.position.y;
//  vector.z = rinput.gripper[0].pose.position.z - rinput.gripper[1].pose.position.z;
tf::Vector3 tfv[2];
for (int i = 0; i < 2; ++i)
{
tf::pointMsgToTF(rinput.gripper[i].pose.position,tfv[i]);
}

return tfv[0] - tfv[1];
}

bool dvrk_wrench::checkLimits(const rvinci_input_msg::rvinci_input r_input)
{
  bool check[2];
  for (int i = 0; i < 2; ++i)
  {
  double xpos = r_input.gripper[i].pose.position.x;
  geometry_msgs::Wrench wrench;
  check[i] = true;
  if(xpos > 0.15 || xpos < -0.15)
  {
    wrench.force.x = -14*(xpos + (1 - 2*i)*0.15);
    dvrk_pub[i].publish(wrench);
    check[i] = false;
  }
 // wrench.force.x = 0;
  //dvrk_pub[i].publish(wrench);
  //dvrk_pub[i].publish(wrench);
  }
if(check[0] & check[1]) return true;
else return false;
}
void dvrk_wrench::publishWrench(const tf::Vector3 vector, const tf::Vector3 orivect[2], double magnitude, tf::Transform tfT[2])
{
  geometry_msgs::Wrench wrench[2];
  strength_[0] = vector_magnitude_ - magnitude;
  double dt = r_->cycleTime().toSec();
  double strengthd = (strength_[0] - strength_[1])/dt;
  double PD = 175*strength_[0] + 0.25*strengthd;

  //normalise, scale, and publish
  for(int i = 0; i<2; ++i)
    {

    tf::Vector3 tvect = tfT[i]*vector;
    tvect *= (1-2*i)*PD;
    tf::Vector3 torque = (ori_magnitude_[i].cross(orivect[i]));
     torqmag_[i] = torque.length();
     if(isnan(torqmag_[i]))
     {torqmag_[i] = torqmago_[i];
       return;
     }
//    torque.normalize();
    double td = (torqmag_[i] - torqmago_[i])/dt;
    double tPD = 1 * torqmag_[i] + 0.001 * td;
    std::cout << td<< std::endl;
    torqmago_[i] = torqmag_[i];
    tf::vector3TFToMsg(tPD*torque,wrench[i].torque);
    tf::vector3TFToMsg(tvect,wrench[i].force);
    dvrk_pub[i].publish(wrench[i]);
    }
    strength_[1] = strength_[0];
}
int main(int argc, char** argv)
{
ros::init(argc, argv, "dvrk_camera_mode");
dvrk_wrench dvrkw;
dvrkw.r_ = new ros::Rate(200);
while(ros::ok())
{
ros::spinOnce();
dvrkw.r_->sleep();
}
delete dvrkw.r_;
}
