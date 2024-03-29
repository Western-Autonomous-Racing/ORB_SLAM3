#include "../include/map_node.hpp"
#include "../include/stereo_inertial_node.hpp"
#include "../include/mono_inertial_node.hpp"
#include "../include/stereo_node.hpp"
#include <signal.h>
#include <stdlib.h>


using namespace std;

void handler(int sig) {
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, handler);
    signal(SIGSEGV, handler);

    rclcpp::init(argc, argv);
    bool bEqual = false;

    rclcpp::executors::MultiThreadedExecutor executor;

    if (strcmp(argv[1], "mono") == 0)
    {
        if (argc == 5 || (argc == 8 && strcmp(argv[5], "--ros-args")) == 0)
        {
            std::string sbEqual(argv[4]);
            if (sbEqual == "true")
                bEqual = true;
        }
        else
        {
            cerr << endl
                 << "Usage: ros2 run orbslam3_ros2 map node_type path_to_vocabulary path_to_settings [do_equalize]" << endl;
            rclcpp::shutdown();
            return 1;
        }

        ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::IMU_MONOCULAR, false); // Instantiate the object directly
    
        auto mono_inertial_node = std::make_shared<MonoInertialNode>(&SLAM, bEqual);
        executor.add_node(mono_inertial_node);

        cout << "Running Mapping..." << endl;
        auto map_node = std::make_shared<MapNode>(&SLAM, argv[3]);
        executor.add_node(map_node);

        executor.spin();
        rclcpp::shutdown();
        return 0;

    }
    else if (strcmp(argv[1], "stereo") == 0)
    {
        if (argc == 6 || (argc == 9 && strcmp(argv[6], "--ros-args") == 0))
        {
            std::string sbEqual(argv[4]);
            if (sbEqual == "true")
                bEqual = true;
        }
        else
        {
            cerr << argv[6] << endl;
            cerr << endl
                 << "Usage: ros2 run orbslam3_ros2 map node_type path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;

            cerr << "You entered: " << endl; 
            
            for (int i = 0; i < argc; i++)
            {
                cerr << argv[i] << " ";
            }
            cerr << endl << "argc: " << argc << endl;
            rclcpp::shutdown();
            return 1;
        }
        std::string sbRect(argv[4]);
        ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::STEREO, true);
        auto stereo_node = std::make_shared<StereoNode>(&SLAM, sbRect == "true", bEqual, argv[3]);
        executor.add_node(stereo_node);

        cout << "Running Mapping..." << endl;
        auto map_node = std::make_shared<MapNode>(&SLAM, argv[3]);
        executor.add_node(map_node);

        executor.spin();
        rclcpp::shutdown();
        return 0;
    }
    else if (strcmp(argv[1], "stereo_inertial") == 0)
    {
        if (argc == 6 || (argc == 9 && strcmp(argv[6], "--ros-args")) == 0)
        {
            std::string sbEqual(argv[4]);
            if (sbEqual == "true")
                bEqual = true;
        }
        else
        {
            cerr << endl
                 << "Usage: ros2 run orbslam3_ros2 map node_type path_to_vocabulary path_to_settings [do_equalize]" << endl;
            rclcpp::shutdown();
            return 1;
        }

        std::string sbRect(argv[4]);
        ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::IMU_STEREO, true);
        auto stereo_inertial_node = std::make_shared<StereoInertialNode>(&SLAM, sbRect == "true", bEqual, argv[3]);
        executor.add_node(stereo_inertial_node);
        cout << "Running Mapping..." << endl;
        auto map_node = std::make_shared<MapNode>(&SLAM, argv[3]);
        executor.add_node(map_node);

        executor.spin();
        rclcpp::shutdown();
        return 0;
    }
    else
    {
        cerr << endl
             << "Error: Invalid node type. " << argv[1] << endl
             << "Usage: ros2 run ORB_SLAM3 map_node node_type path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl
             << "node_type: mono, stereo, stereo_inertial" << endl;
        rclcpp::shutdown();
        return 1;
    }
}