# common 通用组件
# io 输入输出
# core/lio LIO lidar里程计模块
# core/loop_closing 回环检测
# core/g2p5 G2P5 地图匹配模块
# core/maps 瓦片地图
# core/localization 局部定位
# utils 工具
# ui 用户界面
# core/system 系统
# wrapper 包装器

add_library(${PROJECT_NAME}.libs SHARED
        common/nav_state.cc
        common/constant.h
        common/options.cc
        common/params.cc

        io/yaml_io.cc
        io/file_io.cc

        core/lio/eskf.cc
        core/lio/laser_mapping.cc
        core/lio/pointcloud_preprocess.cc
        core/loop_closing/loop_closing.cc

        core/g2p5/g2p5_map.cc
        core/g2p5/g2p5.cc
        core/g2p5/g2p5_subgrid.cc

        core/maps/tiled_map.cc
        core/maps/tiled_map_chunk.cc

        core/localization/localization_result.cc
        core/localization/lidar_loc/lidar_loc.cc
        core/localization/lidar_loc/pclomp/ndt_omp.cpp
        core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp

        core/localization/pose_graph/pgo.cc
        core/localization/pose_graph/pgo_impl.cc
        core/localization/pose_graph/pose_extrapolator.cc
        core/localization/localization.cpp

        utils/timer.cc
        utils/pointcloud_utils.cc

        ui/pangolin_window.cc
        ui/pangolin_window_impl.cc
        ui/ui_car.cc
        ui/ui_cloud.cc
        ui/ui_trajectory.cc

        core/system/slam.cc
        core/system/loc_system.cc

        wrapper/bag_io.cc
)

# 项目依赖多个ros2消息类型与服务
ament_target_dependencies(
        ${PROJECT_NAME}.libs
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "nav_msgs"
        "std_srvs"
        "message_filters"
        "pcl_conversions"
        "tf2_ros"
        "tf2"
)

# 对livox lidar消息和其它自定义消息的支持
add_dependencies(${PROJECT_NAME}.libs
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 集成 miao pcl 与其它第三方库
include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
add_subdirectory(core/miao)

target_link_libraries(${PROJECT_NAME}.libs
        ${third_party_libs}
        ${PCL_LIBRARIES}
        miao.core
        miao.utils
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME}.libs miao.core miao.utils
        DESTINATION lib)

# 应用程序子目录
add_subdirectory(app)
