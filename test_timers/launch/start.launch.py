from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete


def generate_launch_description():

    declared_arguments = []
   
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="When using simulation, you should use simulation time for all nodes too.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="True to start gazebo instead of the clock simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "update_rate",
            default_value="0.0",
            description="if non null use this value for the my_clock update_rate.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "test_timers",
            default_value="false",
            description="True to start timer-based publishers.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "test_rates",
            default_value="false",
            description="True to start rate-based publishers.",
        )
    )
    
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_param_file",
            default_value="gazebo_default_params.yaml",
            choices=["gazebo_default_params.yaml", "gazebo_100HzUpdateRate_params.yaml"],
            description="When using simulation, offers to pass extra ros params to gazebo ros.",
        )
    )

    gazebo_param_file = LaunchConfiguration("gazebo_param_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo = LaunchConfiguration("use_gazebo")
    test_timers = LaunchConfiguration("test_timers")
    test_rates = LaunchConfiguration("test_rates")
    update_rate =  LaunchConfiguration("update_rate")
    

    # timer based node
    timer_node = Node(
        package="test_timers",
        executable="test_timers",
        namespace="timer_based",
        parameters=[{"use_sim_time": use_sim_time}],
        output={'both': 'screen'},
        condition=IfCondition(test_timers),
    )

    # rate based node
    rate_node = Node(
        package="test_timers",
        executable="test_rate",
        namespace="rate_based",
        parameters=[{"use_sim_time": use_sim_time}],
        output={'both': 'screen'},
        condition=IfCondition(test_rates),
    )

    # myclock node, simulates a clock at slow publish rate (100 ms period = 10 Hz)
    myclock = Node(
        package="test_timers",
        executable="myclock",
        output={'both': 'screen'},
        parameters=[{"update_rate": update_rate}],
        condition=UnlessCondition(use_gazebo),
    )

    # Gazebo includes and nodes
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gzserver.launch.py"]
        ),
        launch_arguments={
            "params_file": PathJoinSubstitution([FindPackageShare("test_timers"), "config", gazebo_param_file]),
            "extra_gazebo_args": "--ros-args --log-level debug",
            "verbose": "true"
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gzclient.launch.py"]
        ),
        launch_arguments={
            "verbose": 'true',
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    return LaunchDescription(declared_arguments + [gazebo_gui, gazebo_server, myclock, timer_node, rate_node])

   
