// 节点入口。
#include <ros/ros.h>

#include "hero_chassis_controller/RosPackageTemplate.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hero_chassis_controller");
    ros::NodeHandle node_handle("~");

    hero_chassis_controller::RosPackageTemplate ros_package_template(
        node_handle);

    ros::spin();
    return 0;
}
