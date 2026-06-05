#include <stepit/robot/unitree2/quadruped.h>

namespace stepit {
STEPIT_REGISTER_ROBOTAPI(go2, kDefPriority, RobotApi::make<Go2Api>);
STEPIT_REGISTER_ROBOTAPI(go2w, kDefPriority, RobotApi::make<Go2WApi>);
STEPIT_REGISTER_ROBOTAPI(b2, kDefPriority, RobotApi::make<B2Api>);
STEPIT_REGISTER_ROBOTAPI(aliengo_sim, kDefPriority, RobotApi::make<AliengoSimApi>);
}  // namespace stepit
