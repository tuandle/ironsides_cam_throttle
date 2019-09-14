#include "ironsides_reduce/ironsides_reduce.h"
namespace ironsides_reduce {
int IronsidesReduce::getQueueSize() const {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kQueueSize);
  if (queue_size < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size = 1;
  }
  return queue_size;
}

IronsidesReduce::IronsidesReduce(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh),
      queue_size_(getQueueSize()),
      first_image_sub_(it_, "cam0/image_raw", queue_size_),
      second_image_sub_(it_, "cam1/image_raw", queue_size_),
      first_image_mono_sub_(it_, "cam0/mono/image_raw", queue_size_),
      second_image_mono_sub_(it_, "cam1/mono/image_raw", queue_size_),
      imu_sub_(nh_, "imu0", queue_size_, ros::TransportHints().tcpNoDelay()) {
  nh_private_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be at least 1, set to 1");
    queue_size_ = 1;
  }

  first_image_pub_ = it_.advertise("cam0/throttle/image_raw", queue_size_);
  second_image_pub_ = it_.advertise("cam1/throttle/image_raw", queue_size_);
  first_image_mono_pub_ =
      it_.advertise("cam0/throttle/mono/image_raw", queue_size_);
  second_image_mono_pub_ =
      it_.advertise("cam1/throttle/mono/image_raw", queue_size_);
  imu_cam_sync_ptr_ =
      std::make_shared<message_filters::Synchronizer<ImuCamSyncPolicy>>(
          ImuCamSyncPolicy(queue_size_), imu_sub_, first_image_sub_,
          second_image_sub_, first_image_mono_sub_, second_image_mono_sub_);
  // imu_cam_sync_ptr_->setInterMessageLowerBound(0, ros::Duration(1 * 1e-3));
  imu_cam_sync_ptr_->registerCallback(
      boost::bind(&IronsidesReduce::reduceRate, this, _1, _2, _3, _4, _5));
}

void IronsidesReduce::reduceRate(
    const sensor_msgs::ImuConstPtr& imu_msg_in,
    const sensor_msgs::ImageConstPtr& cam0_in,
    const sensor_msgs::ImageConstPtr& cam1_in,
    const sensor_msgs::ImageConstPtr& cam0_mono_in,
    const sensor_msgs::ImageConstPtr& cam1_mono_in) {
  ROS_INFO_STREAM(std::abs(imu_msg_in->header.stamp.toSec() -
                           cam0_in->header.stamp.toSec()));
  if ((std::abs(imu_msg_in->header.stamp.toSec() -
                cam0_in->header.stamp.toSec())) < 0.00100017) {
    first_image_pub_.publish(cam0_in);
    second_image_pub_.publish(cam1_in);
    first_image_mono_pub_.publish(cam0_mono_in);
    second_image_mono_pub_.publish(cam1_mono_in);
  }
}

}  // namespace ironsides_reduce