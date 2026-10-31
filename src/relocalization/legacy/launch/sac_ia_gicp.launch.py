import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 【本地修复】原代码写的是 get_package_share_directory('sac_ia_gicp') / package='sac_ia_gicp'，
    # 但本工作空间里**没有**名为 sac_ia_gicp 的包（它是 relocalization 包里的一个可执行文件）。
    # 实测原样运行会直接：
    #   PackageNotFoundError: "package 'sac_ia_gicp' not found"
    # 这里改为本包自身。
    package_dir = get_package_share_directory('relocalization')

    # Define node
    sac_ia_gicp_node = Node(
        package='relocalization',
        executable='sac_ia_gicp',
        name='sac_ia_gicp_node',
        parameters=[
            {'target_pcd_file': "/home/zx/getting_relocal/test_map/all_raw_points.pcd"},
            # 【本地修复】与其它重定位节点统一：直接吃 /livox/lidar 的 CustomMsg
            {'pcl_type': 'livox'},
            {'num_threads': 8},
            {'k_serach_source': 100},
            {'k_serach_target': 100},
            # Please tune these param with a clear understanding of the algorithm
            # A good reference would be the official PCL github repo
            {'voxel_grid_leaf_size_source': 0.3},
            {'voxel_grid_leaf_size_target': 0.3},
            {'sac_ia_min_sample_distance': 0.1}, # the minimum distance between samples
            {'sac_ia_correspondence_randomness': 50}, # the number of neighbors to use when selecting a random feature correspondence.
            {'sac_ia_num_samples':5}, # nr_samples the number of samples to use during each iteration
            {'icp_max_correspondence_distance': 1.0},
            {'icp_max_iteration': 10000},
            {'icp_transformation_epsilon': 0.01}, # the transformation epsilon (maximum allowable translation squared difference)
            {'icp_euclidean_fitness_epsilon': 0.01}, # useless
            {'fitness_score_thre':0.1},
            {'mode':0}, # 0 - keep running; 1 - run when fast-lio need recovery
            {'max_optimize_times':3}
        ],
        # 【本地修复】删除原来的 remappings=[('/source_cloud', '/cloud_registered_body')]：
        #   1) /cloud_registered_body 在 FAST-LIVO2 里并不存在（它只发 /cloud_registered），
        #      该重映射指向一个永远没有数据的话题；
        #   2) 现在 pcl_type=livox 走 /livox/lidar，不需要 source_cloud 了。
        # 若仍要用 PointCloud2 输入，把 pcl_type 改成 'standard' 并用 remappings=[('/source_cloud', '<你的话题>')]。
    )

    # Create launch description
    ld = LaunchDescription()

    # Add actions to launch description
    ld.add_action(sac_ia_gicp_node)

    return ld
