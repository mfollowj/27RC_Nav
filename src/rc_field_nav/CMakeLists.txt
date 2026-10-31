cmake_minimum_required(VERSION 3.8)
project(rc_field_nav)

find_package(ament_cmake REQUIRED)

# 仿真资源随包安装；真机启动与脚本在仓库 robot/ 下独立维护。
install(DIRECTORY launch urdf config rviz meshes
        DESTINATION share/${PROJECT_NAME})

# V3 草案场地随包安装，运行时不依赖源码 checkout 或作者的绝对路径。
install(DIRECTORY ../../simulation/worlds/cn_v3_draft
        DESTINATION share/${PROJECT_NAME}/worlds
        USE_SOURCE_PERMISSIONS)

# 建图产出的地图目录（先建空目录，保证 install 后路径存在）
install(DIRECTORY maps
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL)

install(PROGRAMS
        # ---- 仿真底盘：physics 轮速驱动 + 键盘遥控 ----
        scripts/omni_drive.py
        scripts/omni_teleop.py
        # ---- 建图与存图 ----
        scripts/record_trajectory.py
        scripts/save_pointcloud_pcd.py
        scripts/save_cloud_pcd.py
        scripts/map3d_to_costmap.py
        scripts/map_sanity_check.py
        scripts/save_prior_map.sh
        # ---- 先验地图导航（导航中持续重定位 -> map->odom） ----
        scripts/prior_map_localizer.py
        scripts/nav2_supervisor.py
        # ---- 清场 ----
        scripts/kill_sim.sh
        DESTINATION lib/${PROJECT_NAME}
        OPTIONAL)

ament_package()
