#include "drive_ros_imu_filter/broadcast_transform.h"


static geometry_msgs::Transform current_trafo;
static std::string target_frame;
static std::string source_frame;
static std::mutex trafo_mu;

tf2_ros::StaticTransformBroadcaster* tf2_broadcast;

const static int tf_pub_rate = 25;

enum{
  FAILURE = 0,
  SUCCESS = 1
};

// when a new imu message arrives
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

  if(source_frame.empty())
  {
    source_frame = msg->header.frame_id;
    ROS_INFO_STREAM("Got first IMU msg. Setting source frame to: " << source_frame);
    return;
  }

}

// loads the default trafo from parameter server
bool load_default_trafo(const ros::NodeHandle* pnh)
{
  bool ok = SUCCESS;
  double x,y,z;

  ok &= pnh->getParam("default_tf_param/x_trans", x);
  ok &= pnh->getParam("default_tf_param/y_trans", y);
  ok &= pnh->getParam("default_tf_param/z_trans", z);

  double roll,pitch,yaw;
  ok &= pnh->getParam("default_tf_param/roll", roll);
  ok &= pnh->getParam("default_tf_param/pitch", pitch);
  ok &= pnh->getParam("default_tf_param/yaw", yaw);

  if(FAILURE == ok)
  {
    ROS_ERROR("Problem occured on loading parameters!");
    return FAILURE;
  }


  trafo_mu.lock();
  current_trafo.translation.x = x;
  current_trafo.translation.y = y;
  current_trafo.translation.z = z;

  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  current_trafo.rotation.w = q.w();
  current_trafo.rotation.x = q.x();
  current_trafo.rotation.y = q.y();
  current_trafo.rotation.z = q.z();
  trafo_mu.unlock();

  ROS_INFO_STREAM("Loaded trafo params [x,y,z,roll,pitch,yaw]: "
                                         << x << ", "
                                         << y << ", "
                                         << z << ", "
                                         << roll << ", "
                                         << pitch << ", "
                                         << yaw);

  return SUCCESS;
}

// broadcast tf
void tf_broadcast(void)
{
  ros::Rate r(tf_pub_rate);
  while(ros::ok())
  {
    if(!source_frame.empty())
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = source_frame;
      transformStamped.child_frame_id = target_frame;

      trafo_mu.lock();
      transformStamped.transform = current_trafo;
      trafo_mu.unlock();

      tf2_broadcast->sendTransform(transformStamped);
    }
    r.sleep();
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_broadcast_transform");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // load default trafo param
  if(!load_default_trafo(&pnh)){
    return 1;
  }

  // setup tf 2 stuff
  tf2_broadcast = new tf2_ros::StaticTransformBroadcaster();

  // get target frame
  target_frame = pnh.param<std::string>("target_frame", "");
  ROS_INFO_STREAM("Loaded target frame: " << target_frame);

  // clear source frame (will be filled when first imu message arrives)
  source_frame.clear();

  // setup subscriber and publisher
  ros::Subscriber imu_sub = pnh.subscribe("imu_in", 10, imuCallback);

  // start separate thread
  std::thread tf_br(tf_broadcast);

  // forever loop
  while(ros::ok()){
    ros::spin();
  }

  return 0;
}
