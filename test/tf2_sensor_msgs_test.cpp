#include <gtest/gtest.h>
#include <stdio.h>
#include <ros/ros.h>

#include "drive_ros_msgs/tf2_IMU.h"


TEST(imu_transfom_test, doTransform)
{
  sensor_msgs::Imu imu_in;
  geometry_msgs::TransformStamped t_in; // from imu to vehicle

  // only rotation around z
  imu_in.angular_velocity.z = 1;
  t_in.transform.translation.x = -1;
  {
    sensor_msgs::Imu imu_out;
    tf2::doTransform(imu_in, imu_out, t_in);
    EXPECT_FLOAT_EQ(0, imu_out.angular_velocity.x);
    EXPECT_FLOAT_EQ(0, imu_out.angular_velocity.y);
    EXPECT_FLOAT_EQ(1, imu_out.angular_velocity.z);

    EXPECT_FLOAT_EQ(1, imu_out.linear_acceleration.x);
    EXPECT_FLOAT_EQ(0, imu_out.linear_acceleration.y);
    EXPECT_FLOAT_EQ(0, imu_out.linear_acceleration.z);
  }

  // add some accelerations to imu in
  imu_in.linear_acceleration.x = -10;
  imu_in.linear_acceleration.y = -3;
  imu_in.linear_acceleration.z =  5;
  {
    sensor_msgs::Imu imu_out;
    tf2::doTransform(imu_in, imu_out, t_in);
    EXPECT_FLOAT_EQ(0, imu_out.angular_velocity.x);
    EXPECT_FLOAT_EQ(0, imu_out.angular_velocity.y);
    EXPECT_FLOAT_EQ(1, imu_out.angular_velocity.z);

    EXPECT_FLOAT_EQ(-9, imu_out.linear_acceleration.x);
    EXPECT_FLOAT_EQ(-3, imu_out.linear_acceleration.y);
    EXPECT_FLOAT_EQ( 5, imu_out.linear_acceleration.z);
  }

}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "tf2_sensor_msgs_imu");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
