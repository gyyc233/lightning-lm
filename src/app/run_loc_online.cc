//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试

// 在线版需要ROS2环境和实时数据流，离线版只要bag包
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    // 初始化ROS2客户端
    rclcpp::init(argc, argv);

    // 定位系统初始化
    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    /// 默认起点开始定位
    loc.SetInitPose(SE3());
    loc.Spin();

    rclcpp::shutdown();

    return 0;
}