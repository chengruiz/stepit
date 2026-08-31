#include <stepit/spin.h>
#include <stepit/ros2/node.h>

namespace stepit {
class Ros2Spin : public WaitForSigInt {
 public:
  int spin() override {
    rclcpp::Rate rate(1000);
    while (not sigint_received_) {
      rclcpp::spin_some(getNode());
      rate.sleep();
    }
    return 0;
  }
};

STEPIT_REGISTER_SPIN(ros2, kDefPriority, Spin::make<Ros2Spin>);
}  // namespace stepit
