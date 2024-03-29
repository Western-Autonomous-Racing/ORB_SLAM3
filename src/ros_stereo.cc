#include "../include/stereo_node.hpp"

using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  bool bEqual = false;
  if (argc < 4 || argc > 5)
  {
    cerr << endl
         << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    rclcpp::shutdown();
    return 1;
  }

  cout << argv[2] << endl;

  std::string sbRect(argv[3]);
  if (argc == 5)
  {
    std::string sbEqual(argv[4]);
    if (sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
  auto stereo_node = std::make_shared<StereoNode>(&SLAM, sbRect == "true", bEqual, argv[2]);

  rclcpp::spin(stereo_node);
  rclcpp::shutdown();

  return 0;
}