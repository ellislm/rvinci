#include <ros/ros.h>
#include <rvinci_input_msg/rvinci_input.h>
#include <geometry_msgs/Wrench.h>
#include <cmath>

class dvrk_wrench
{
public:
  dvrk_wrench();
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);
  double vector_magnitude_;
  ros::Rate* r_;
private:
  ros::NodeHandle n;
  ros::Subscriber rvinci_sub;
  ros::Publisher dvrk_pub[2];
  double strength_[2]; //current and previous
  void publishWrench(const geometry_msgs::Vector3, double);
  geometry_msgs::Vector3 getVector(const rvinci_input_msg::rvinci_input);
  bool checkLimits(const rvinci_input_msg::rvinci_input);
  double getMagnitude(const geometry_msgs::Vector3);
};

dvrk_wrench::dvrk_wrench()
{
  rvinci_sub = n.subscribe<rvinci_input_msg::rvinci_input>("/rvinci_input_update",10,&dvrk_wrench::inputCallback,this);
  dvrk_pub[0] = n.advertise<geometry_msgs::Wrench>("/dvrk_mtml/set_wrench",10);
  dvrk_pub[1] = n.advertise<geometry_msgs::Wrench>("/dvrk_mtmr/set_wrench",10);
}
void dvrk_wrench::inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input)
{
  geometry_msgs::Vector3 curvect = getVector(*r_input);
 // if (checkLimits(*r_input))
 // {
  if(!r_input->camera && getMagnitude(curvect) >  0.12)
  {
  vector_magnitude_ = getMagnitude(curvect);
  publishWrench(curvect,vector_magnitude_);
  }
  else
  {
    double curmag = getMagnitude(curvect);
    publishWrench(curvect, curmag);
  }
 // }
}
geometry_msgs::Vector3 dvrk_wrench::getVector(const rvinci_input_msg::rvinci_input rinput)
{
  geometry_msgs::Vector3 vector;
  vector.x = rinput.gripper[0].pose.position.x - rinput.gripper[1].pose.position.x;
  vector.y = rinput.gripper[0].pose.position.y - rinput.gripper[1].pose.position.y;
  vector.z = rinput.gripper[0].pose.position.z - rinput.gripper[1].pose.position.z;
  return vector;
}
double dvrk_wrench::getMagnitude(const geometry_msgs::Vector3 vector)
{
  return sqrt(pow(vector.x,2) + pow(vector.y,2) + pow(vector.z,2));
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
void dvrk_wrench::publishWrench(const geometry_msgs::Vector3 vector, double magnitude)
{
  geometry_msgs::Wrench wrench[2];
  strength_[0] = vector_magnitude_ - magnitude;
  double strengthd = (strength_[0] - strength_[1])/r_->cycleTime().toSec();
  double PD = 9*strength_[0] + 0.01*strengthd;

  //normalise, scale, and publish
  for(int i = 0; i<2; ++i)
    {
    wrench[i].force.x = (1 - 2*i)*vector.x*PD/magnitude;
    wrench[i].force.y = (1 - 2*i)*vector.y*PD/magnitude;
    wrench[i].force.z = (1 - 2*i)*vector.z*PD/magnitude;
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
