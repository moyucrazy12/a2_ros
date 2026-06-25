"""
Autonomous exploration launch for A2 simulation using TARE planner.
TUNED FOR TIGHT MAZE: ~10x10 m arena, 1.5 m corridors, 2-3 m rooms, 90-deg
corners, dead-ends, flat floor, fully static. Lines changed from the baseline
are marked "# tuned: was <old> - <reason>".

Pair this with the maze TARE config (tare_planner_maze.yaml): point tare_config
at it, or keep config_2.yaml if you prefer.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    description_dir = get_package_share_directory('a2_description')
    a2_ros_dir      = get_package_share_directory('a2_ros')
    rviz_path        = os.path.join(a2_ros_dir, 'rviz', 'exploration.rviz')
    tare_config      = os.path.join(a2_ros_dir, 'config', 'autonomy', 'config_2.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    nodes = [
        rviz_arg,
        SetParameter(name='use_sim_time', value=False),

        # ---- terrain analysis (local map) ----
        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[{
                'scanVoxelSize':       0.05,
                'decayTime':           20.0,
                'noDecayDis':          5.0,
                'clearingDis':         25.0,
                'useSorting':          True,
                'quantileZ':           0.25,
                'considerDrop':        False,  # tuned: was True - maze floor is flat with no real drops; avoids phantom "negative obstacle" points from floor noise
                'limitGroundLift':     True,
                'maxGroundLift':       0.25,
                'clearDyObs':          False,  # keep False - environment is fully static; never clear wall returns
                'minDyObsDis':         0.3,
                'minDyObsAngle':       0.0,
                'minDyObsRelZ':        -0.5,
                'absDyObsRelZThre':    0.2,
                'minDyObsVFOV':        -16.0,
                'maxDyObsVFOV':        16.0,
                'minDyObsPointNum':    1,
                'noDataObstacle':      False,  # keep False - unknown cells must stay open so TARE can drive into frontiers
                'noDataBlockSkipNum':  0,
                'minBlockPointNum':    10,
                'vehicleHeight':       0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 2.0,
                'minRelZ':             -1.0,
                'maxRelZ':             1.0,
                'disRatioZ':           0.2,
            }],
        ),

        # ---- terrain analysis ext (global map) ----
        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrainAnalysisExt',
            output='screen',
            parameters=[{
                'scanVoxelSize':        0.1,
                'decayTime':            25.0,
                'noDecayDis':           0.0,
                'clearingDis':          35.0,
                'useSorting':           True,
                'quantileZ':            0.25,
                'vehicleHeight':        0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre':  2.0,
                'lowerBoundZ':          -1.0,
                'upperBoundZ':          1.0,
                'disRatioZ':            0.1,
                'checkTerrainConn':     True,
                'terrainUnderVehicle':  -0.75,
                'terrainConnThre':      0.5,
                'ceilingFilteringThre': 2.0,
                'localTerrainMapRadius': 5.0,  # tuned: was 4.0 - small arena; trust the fine 0.2 m local /terrain_map over more of the maze instead of the coarse 0.4 m ext cells
            }],
        ),
        # ---- local planner ----
        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            parameters=[{
                'pathFolder':          get_package_share_directory('local_planner') + '/paths',
                'vehicleLength':       0.7,
                'vehicleWidth':        0.45,
                'sensorOffsetX':       0.0,
                'sensorOffsetY':       0.0,
                'twoWayDrive':         False,
                'laserVoxelSize':      0.05,
                'terrainVoxelSize':    0.2,
                'useTerrainAnalysis':  True,
                'checkObstacle':       True,
                'checkRotObstacle':    True,   # keep True - rejects in-place turns that would clip a wall (footprint diag ~1.0 m vs 1.5 m corridor)
                'adjacentRange':       2.0,
                'obstacleHeightThre':  0.2,    # tuned: was 0.25 - cleanly above the flat floor but a touch more conservative so wall bases always block
                'groundHeightThre':    0.1,
                'costHeightThre':      0.1,
                'costScore':           0.02,
                'useCost':             False,
                'pointPerPathThre':    2,
                'minRelZ':             -0.5,
                'maxRelZ':             0.8,
                'maxSpeed':            1.2,
                'dirWeight':           0.05,   # tuned: was 0.1 - lower goal-direction penalty so the planner will commit to corridors that initially head away from the goal (turns at junctions / into dead-ends)
                'dirThre':             90.0,
                'dirToVehicle':        False,
                'pathScale':           0.5,
                'minPathScale':        0.3,
                'pathScaleStep':       0.25,
                'pathScaleBySpeed':    False,
                'minPathRange':        1.0,
                'pathRangeStep':       0.1,
                'pathRangeBySpeed':    True,
                'pathCropByGoal':      True,
                'autonomyMode':        True,
                'autonomySpeed':       0.5,
                'joyToSpeedDelay':     2.0,
                'joyToCheckObstacleDelay': 5.0,
                'goalClearRange':      0.4,
                'goalX':               0.0,
                'goalY':               0.0,
            }],
        ),

        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[{
                'sensorOffsetX':    0.0,
                'sensorOffsetY':    0.0,
                'pubSkipNum':       1,
                'twoWayDrive':      False,
                'lookAheadDis':     0.6,   # tuned: was 1.0 - shorter look-ahead tracks 90-deg corners tightly instead of cutting them into the walls
                'yawRateGain':      4.0,
                'stopYawRateGain':  4.0,   # tuned: was 3.0 - quicker in-place alignment when turning at a junction or reversing out of a dead-end
                'maxYawRate':       45.0,
                'maxSpeed':         1.2,
                'maxAccel':         2.0,
                'switchTimeThre':   1.0,
                'dirDiffThre':      0.2,   # tuned: was 0.3 - require tighter heading alignment (~11 deg) before driving forward so it doesn't scrape a wall on corner exit
                'stopDisThre':      0.3,
                'slowDwnDisThre':   0.6,
                'useInclRateToSlow': False,
                'inclRateThre':     120.0,
                'slowRate1':        0.25,
                'slowRate2':        0.5,
                'slowTime1':        2.0,
                'slowTime2':        2.0,
                'useInclToStop':    False,
                'inclThre':         45.0,
                'stopTime':         5.0,
                'noRotAtStop':      False,
                'noRotAtGoal':      True,
                'autonomyMode':     True,
                'autonomySpeed':    0.5,
                'joyToSpeedDelay':  2.0,
            }],
        ),
        # Terrain map acummulator

        Node(
            package='terrain_map_accumulator',
            executable='terrain_map_accumulator',
            name='terrain_map_accumulator_node',
            output='screen',
        ),

        # ---- TARE planner (autonomous exploration) ----
        Node(
            package='tare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='screen',
            parameters=[tare_config],
        ),


        # ---- RViz ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]

    return LaunchDescription(nodes)