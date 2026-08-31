#include <stepit/spin.h>
#include <stepit/ros/node_handle.h>

namespace stepit {
class RosSpin : public WaitForSigInt {
 public:
  int spin() override {
    ros::Rate rate(1000);
    while (not sigint_received_) {
      ros::spinOnce();
      rate.sleep();
    }
    ros::shutdown();
    return 0;
  }
};

STEPIT_REGISTER_SPIN(ros, kDefPriority, Spin::make<RosSpin>);
}  // namespace stepit
