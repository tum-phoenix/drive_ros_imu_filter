#include "drive_ros_imu_filter/broadcast_transform.h"


static geometry_msgs::Transform current_trafo;
static geometry_msgs::Vector3 current_gyro_off;
static std::string vehicle_frame;
static std::string imu_frame;
static std::mutex trafo_mu;


static int no_motion_ct;

static sensor_msgs::Imu msg_old;
static std::vector<sensor_msgs::Imu> msg_buffer;

tf2_ros::StaticTransformBroadcaster* tf2_broadcast;

ros::Publisher gyro_off_pub;

const static int tf_pub_rate = 25;


const double* no_motion_acc_thres;
const double* no_motion_gyro_thres;
const int* buffer_length;

const bool* enable_autocali;

const bool* output_diff;

const float GRAVITY = 9.81;

enum{
  FAILURE = 0,
  SUCCESS = 1
};

enum{
  MOTION= 0,
  NO_MOTION = 1
};


// detects if there is no motion in the msg
bool detectNoMotion(const sensor_msgs::Imu::ConstPtr &msg)
{
  bool ret = NO_MOTION;

  ret &= (*no_motion_gyro_thres
          > std::fabs(static_cast<float>(msg->angular_velocity.x - msg_old.angular_velocity.x)));
  ret &= (*no_motion_gyro_thres
          > std::fabs(static_cast<float>(msg->angular_velocity.y - msg_old.angular_velocity.y)));
  ret &= (*no_motion_gyro_thres
          > std::fabs(static_cast<float>(msg->angular_velocity.z - msg_old.angular_velocity.z)));

  ret &= (*no_motion_acc_thres
          > std::fabs(static_cast<float>(msg->linear_acceleration.x - msg_old.linear_acceleration.x)));
  ret &= (*no_motion_acc_thres
          > std::fabs(static_cast<float>(msg->linear_acceleration.y - msg_old.linear_acceleration.y)));
  ret &= (*no_motion_acc_thres
          > std::fabs(static_cast<float>(msg->linear_acceleration.z - msg_old.linear_acceleration.z)));


  if(*output_diff)
  {
    ROS_INFO_STREAM(   " gyro_x: " << std::fabs(static_cast<float>(msg->angular_velocity.x - msg_old.angular_velocity.x))
                    << " gyro_y: " << std::fabs(static_cast<float>(msg->angular_velocity.y - msg_old.angular_velocity.y))
                    << " gyro_z: " << std::fabs(static_cast<float>(msg->angular_velocity.z - msg_old.angular_velocity.z))
                    << " acc_x: "  << std::fabs(static_cast<float>(msg->linear_acceleration.x - msg_old.linear_acceleration.x))
                    << " acc_y: "  << std::fabs(static_cast<float>(msg->linear_acceleration.y - msg_old.linear_acceleration.y))
                    << " acc_z: "  << std::fabs(static_cast<float>(msg->linear_acceleration.z - msg_old.linear_acceleration.z)));
  }



  return ret;
}

// calculates the transformation rotation and gyro offsets based on values in message buffer
bool calculateOffsets(void)
{
  geometry_msgs::Transform new_trafo;
  geometry_msgs::Vector3 new_gyro_off;

  // keep translation
  new_trafo.translation = current_trafo.translation;

  // get Euler angles from msg
  double roll, pitch, yaw;
  tf::Quaternion q_old;
  tf::quaternionMsgToTF(current_trafo.rotation, q_old);
  tf::Matrix3x3(q_old).getRPY(roll, pitch, yaw);

  // get mean values
  float a_x_mean = 0;
  float a_y_mean = 0;
  float a_z_mean = 0;

  // loop over message buffer
  for(auto it = msg_buffer.begin(); it != msg_buffer.end(); ++it)
  {
    a_x_mean += it->linear_acceleration.x;
    a_y_mean += it->linear_acceleration.y;
    a_z_mean += it->linear_acceleration.z;
    new_gyro_off.x += it->angular_velocity.x;
    new_gyro_off.y += it->angular_velocity.y;
    new_gyro_off.z += it->angular_velocity.z;
  }

  a_x_mean = a_x_mean / (float) msg_buffer.size();
  a_y_mean = a_y_mean / (float) msg_buffer.size();
  a_z_mean = a_z_mean / (float) msg_buffer.size();

  new_gyro_off.x = new_gyro_off.x / (float) msg_buffer.size();
  new_gyro_off.y = new_gyro_off.y / (float) msg_buffer.size();
  new_gyro_off.z = new_gyro_off.z / (float) msg_buffer.size();


  // keep yaw (without good magnetometer data there is no way to estimate this)
  yaw = yaw;

  // get pitch
  pitch = -asin(a_x_mean / GRAVITY);

  // get roll
  roll = atan(a_y_mean / a_z_mean);

  // write Euler angles to new trafo
  tf::Quaternion q_new;
  q_new.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q_new, new_trafo.rotation);


  // write as new trafo and gyro off
  trafo_mu.lock();
  current_trafo = new_trafo;
  current_gyro_off = new_gyro_off;
  trafo_mu.unlock();

  return true;
}


// when a new imu message arrives
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

  if(imu_frame.empty())
  {
    imu_frame = msg->header.frame_id;
    ROS_INFO_STREAM("Got first IMU msg. Setting IMU frame to: " << imu_frame);
    return;
  }


  if(NO_MOTION == detectNoMotion(msg))
  {
    no_motion_ct ++;
    msg_buffer.push_back(*msg);

    if(*buffer_length < msg_buffer.size() && enable_autocali)
    {
      ROS_DEBUG_STREAM("Detected no motion for " << msg_buffer.size() <<" messages.");
      ROS_DEBUG_STREAM("Start autocalibration...");

      // recalculate msg buffer and gyro offset
      calculateOffsets();

      // reset no motion counter and buffer
      no_motion_ct = 0;
      msg_buffer.clear();
    }

  }else{
    no_motion_ct = 0;
    msg_buffer.clear();
  }


  msg_old = *msg;
}

// loads the default trafo and gyro off from parameter server
bool load_default(const ros::NodeHandle* pnh)
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


  current_gyro_off.x = 0;
  current_gyro_off.y = 0;
  current_gyro_off.z = 0;
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

// broadcast tf and publish gyro offset
void publish(void)
{
  ros::Rate r(tf_pub_rate);
  while(ros::ok())
  {
    if(!imu_frame.empty())
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = vehicle_frame;
      transformStamped.child_frame_id = imu_frame;

      trafo_mu.lock();
      transformStamped.transform = current_trafo;
      geometry_msgs::Vector3 gyro_off(current_gyro_off);
      trafo_mu.unlock();

      gyro_off_pub.publish(gyro_off);

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
  if(!load_default(&pnh)){
    return 1;
  }

  // setup tf 2 stuff
  tf2_broadcast = new tf2_ros::StaticTransformBroadcaster();

  // get vehicle frame
  vehicle_frame = pnh.param<std::string>("vehicle_frame", "");
  ROS_INFO_STREAM("Loaded vehicle frame: " << vehicle_frame);

  // get calibration parameters
  no_motion_acc_thres =  new const double(pnh.param<double>("no_motion_acc_thres", 0));
  no_motion_gyro_thres = new const double(pnh.param<double>("no_motion_gyro_thres", 0));
  buffer_length =        new const int(pnh.param<int>("buffer_length", 100));
  enable_autocali =      new const bool(pnh.param<bool>("enable_autocali", true));
  output_diff =          new const bool(pnh.param<bool>("output_diff", false));

  // clear source frame (will be filled when first imu message arrives)
  imu_frame.clear();

  // clear transformation buffer
  msg_buffer.clear();

  // reset no motion counter
  no_motion_ct = 0;

  // setup subscriber and publisher
  ros::Subscriber imu_sub = pnh.subscribe("imu_in", 10, imuCallback);

  // create gyro offset publisher
  gyro_off_pub = pnh.advertise<geometry_msgs::Vector3>("gyro_off", 10);

  // start separate thread
  std::thread tf_br(publish);

  // forever loop
  while(ros::ok()){
    ros::spin();
  }

  // clean up
  delete no_motion_acc_thres;
  delete no_motion_gyro_thres;
  delete buffer_length;

  return 0;
}
