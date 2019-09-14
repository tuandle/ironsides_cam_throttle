#ifndef IRONSIDE_REDUCE_H
#define IRONSIDE_REDUCE_H

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/pass_through.h>
#include <message_filters/simple_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <cmath>

namespace ironsides_reduce {

constexpr int kQueueSize = 10;
class IronsidesReduce {
 public:
  IronsidesReduce(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void reduceRate(const sensor_msgs::ImuConstPtr& imu_msg_in,
                  const sensor_msgs::ImageConstPtr& cam0_in,
                  const sensor_msgs::ImageConstPtr& cam1_in,
                  const sensor_msgs::ImageConstPtr& cam0_mono_in,
                  const sensor_msgs::ImageConstPtr& cam1_mono_in);

 private:
  int getQueueSize() const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
  image_transport::ImageTransport it_;

  // subscribers
  image_transport::SubscriberFilter first_image_sub_;
  image_transport::SubscriberFilter second_image_sub_;
  image_transport::SubscriberFilter first_image_mono_sub_;
  image_transport::SubscriberFilter second_image_mono_sub_;

  image_transport::Publisher first_image_pub_;
  image_transport::Publisher second_image_pub_;
  image_transport::Publisher first_image_mono_pub_;
  image_transport::Publisher second_image_mono_pub_;

  /*typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Imu, sensor_msgs::Image, sensor_msgs::Image>
      ImuCamSyncPolicy;*/
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Imu, sensor_msgs::Image, sensor_msgs::Image,
      sensor_msgs::Image, sensor_msgs::Image>
      ImuCamSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ImuCamSyncPolicy>>
      imu_cam_sync_ptr_;

  int queue_size_;
  sensor_msgs::ImuPtr imu_queue_;
};

}  // namespace ironsides_reduce

#endif