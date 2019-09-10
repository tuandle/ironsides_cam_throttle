#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ironsides_reduce/ironsides_reduce.h"

namespace ironsides_reduce {

class IronsidesReduceNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      ironsides_reduce_ = std::make_shared<IronsidesReduce>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<IronsidesReduce> ironsides_reduce_;
};
}  // namespace ironsides_reduce

PLUGINLIB_EXPORT_CLASS(ironsides_reduce::IronsidesReduceNodelet,
                       nodelet::Nodelet);
