#include "drive_ros_imu_filter/apply_transform.h"

static std::string target_frame;

ros::Publisher imu_pub;
tf2_ros::Buffer* tf2_buffer;
tf2_ros::TransformListener* tf2_listener;

bool received_first_msg = false;


void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

  if(!received_first_msg)
  {
    ROS_INFO("Received first IMU message. Wait 1s to let the broadcaster some time to publish");
    ros::Duration d(1);
    d.sleep();
    received_first_msg = true;
  }


  try
  {
    sensor_msgs::Imu imu_out;
    tf2_buffer->transform(*msg, imu_out, target_frame);
    imu_pub.publish(imu_out);
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR_STREAM("IMU Transform failure: " << ex.what());
    return;
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_apply_transform");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // setup tf 2 stuff
  tf2_buffer = new tf2_ros::Buffer();
  tf2_listener = new tf2_ros::TransformListener(*tf2_buffer);


  // get target frame
  target_frame = pnh.param<std::string>("target_frame", "");
  ROS_INFO_STREAM("Loaded target frame: " << target_frame);

  // setup subscriber and publisher
  ros::Subscriber imu_sub = pnh.subscribe("imu_in", 10, imuCallback);
  imu_pub = pnh.advertise<sensor_msgs::Imu>("imu_out", 10);

  // forever loop
  while(ros::ok()){
    ros::spin();
  }

  return 0;
}
