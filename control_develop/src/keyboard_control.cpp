#include "control_develop/keyboard_control.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher");
  keyboard_control M;
  M.main();
}