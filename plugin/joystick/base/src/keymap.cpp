#include <stepit/joystick/keymap.h>

namespace stepit {
namespace joystick {
Keymap::Keymap(const yml::Node &config) {
  config["A"].to(A);
  config["B"].to(B);
  config["X"].to(X);
  config["Y"].to(Y);
  config["LB"].to(LB);
  config["RB"].to(RB);
  config["Select"].to(Select);
  config["Start"].to(Start);
  config["LAS"].to(LAS);
  config["RAS"].to(RAS);
  config["Up"].to(Up);
  config["Down"].to(Down);
  config["Left"].to(Left);
  config["Right"].to(Right);

  config["las_x"].to(las_x);
  config["las_y"].to(las_y);
  config["ras_x"].to(ras_x);
  config["ras_y"].to(ras_y);
  config["lt"].to(lt);
  config["rt"].to(rt);
}
}  // namespace joystick
}  // namespace stepit
