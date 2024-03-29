#include "../include/mono_inertial_node.hpp"

using namespace std;

int main(int argc, char **argv)
{
  // rclcpp::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, rclcpp::console::levels::Info);
  bool bEqual = false;
  if (argc < 3 || argc > 4)
  {
    cerr << endl
         << "Usage: ros2 run ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    rclcpp::shutdown();
    return 1;
  }

  if (argc == 4)
  {
    std::string sbEqual(argv[3]);
    if (sbEqual == "true")
      bEqual = true;
  }

  rclcpp::init(argc, argv);

  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
  auto mono_inertial_node = std::make_shared<MonoInertialNode>(&SLAM, bEqual);

  rclcpp::spin(mono_inertial_node);

  rclcpp::shutdown();

  return 0;
}
