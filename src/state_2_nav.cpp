#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "random_numbers/random_numbers.h"

#include <sstream>

int publish = 0;
ros::Publisher *p_pub;

std::string model;
double update_freq;
double std_dev_position;
double std_dev_orientation;

geometry_msgs::PoseWithCovarianceStamped pose;

random_numbers::RandomNumberGenerator rnum;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if (publish) {
        pose.header.seq++;
        pose.header.stamp = ros::Time::now();
        int indx = 0;
        for (indx = 0; indx < msg->name.size(); indx++) {
            if (msg->name[indx] == model){
                break;
            }
        }

        pose.pose.pose = msg->pose[indx];
        pose.pose.pose.position.x += rnum.gaussian(0.0, std_dev_position);
        pose.pose.pose.position.y += rnum.gaussian(0.0, std_dev_position);
        pose.pose.pose.position.z += rnum.gaussian(0.0, std_dev_position);

        pose.pose.pose.orientation.x += rnum.gaussian(0.0, std_dev_orientation);
        pose.pose.pose.orientation.y += rnum.gaussian(0.0, std_dev_orientation);
        pose.pose.pose.orientation.z += rnum.gaussian(0.0, std_dev_orientation);

        double mag = sqrt(pose.pose.pose.orientation.x*pose.pose.pose.orientation.x + \
            pose.pose.pose.orientation.y*pose.pose.pose.orientation.y + \
            pose.pose.pose.orientation.z*pose.pose.pose.orientation.z);

        pose.pose.pose.orientation.x /= mag;
        pose.pose.pose.orientation.y /= mag;
        pose.pose.pose.orientation.z /= mag;
        pose.pose.pose.orientation.w = 1.0 - mag;

        pose.pose.covariance = boost::array<double, 36UL> {std_dev_position, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                                            0.0, std_dev_position, 0.0, 0.0, 0.0, 0.0, \
                                                            0.0, 0.0, std_dev_position, 0.0, 0.0, 0.0, \
                                                            0.0, 0.0, 0.0, std_dev_orientation, 0.0, 0.0, \
                                                            0.0, 0.0, 0.0, 0.0, std_dev_orientation, 0.0, \
                                                            0.0, 0.0, 0.0, 0.0, 0.0, std_dev_orientation};

                                                            
        p_pub->publish(pose);          
        publish = 0;                                                  
    }

}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "state_2_nav");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::NodeHandle nh("~");

  pose = geometry_msgs::PoseWithCovarianceStamped();

  ros::Duration(1.0).sleep();
  if (!nh.getParam("model", model)) {
    ROS_ERROR("No model named");
    n.shutdown();
    exit(-1);
  }

  update_freq = nh.param("update_freq", 10);
  std_dev_position = nh.param("std_dev_position", 0.0);
  std_dev_orientation = nh.param("std_dev_orientation", 0.0);
  pose.header.frame_id = nh.param("frame_id", std::string("base_link"));


  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  
  ros::Publisher pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("gazebo_pose", 1000);
  p_pub = &pub;
  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1000, stateCallback);

  ros::Rate loop_rate(update_freq);

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    publish = 1;
    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}