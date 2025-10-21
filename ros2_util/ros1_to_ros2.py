import sys

import re

def snake_case(s):
    s = re.sub(r'([a-z])([A-Z])', r'\1_\2', s) # Insert underscore between lowercase and uppercase letters
    return s.lower()

msg_replace = [
    ["std", "Bool"],   # 4th arg implicit snake_case, 3rd arg implicit "msg"
    ["std", "Empty"],
    ["std", "Float32"],
    ["std", "Float64"],
    ["std", "Int8"],
    ["std", "Int16"],
    ["std", "Int32"],
    ["std", "Int64"],
    ["std", "Header"],
    ["std", "ColorRGBA"],
    ["geometry", "Accel"],
    ["geometry", "AccelStamped"],
    ["geometry", "Pose"],
    ["geometry", "PoseArray"],
    ["geometry", "PoseStamped"],
    ["geometry", "Twist"],
    ["geometry", "TwistStamped"],
    ["geometry", "Point"],
    ["geometry", "PointStamped"],
    ["geometry", "Transform"],
    ["geometry", "TransformStamped"],
    ["geometry", "Quaternion"],
    ["geometry", "Vector3"],
    ["sensor", "CameraInfo"],
    ["sensor", "JointState"],
    ["sensor", "Image"],
    ["sensor", "Imu"],
    ["sensor", "LaserScan"],
    ["sensor", "PointField"],
    ["sensor", "PointCloud2"],
    ["nav", "Odometry"],
    ["nav", "Path"],
    ["visualization", "Marker"],
    ["visualization", "MarkerArray"],
    ["trajectory", "JointTrajectory"],
    ["trajectory", "JointTrajectoryPoint"],
    ["control", "FollowJointTrajectoryAction", 'action', 'follow_joint_trajectory'],
    ["control", "GripperCommandAction", 'action', 'gripper_command'],
    ["rosgraph", "Clock"],
    ["dynamixel_workbench", "DynamixelStateList"],
    ["dynamixel_workbench", "DynamixelState"],
    ["dynamixel_workbench", "DynamixelCommand", 'srv'],
]

def msg_replaces(s):
    results = []
    for r in s:
        if len(r) < 3:
            r.append('msg')
        if len(r) < 4:
            r.append(snake_case(r[1]))
        include_orig = f"<{r[0]}_msgs/{r[1]}.h>"
        include_new = f"<{r[0]}_msgs/{r[2]}/{r[3]}.hpp>"
        results.append([include_orig, include_new])
        include_orig = f'"{r[0]}_msgs/{r[1]}.h"'
        include_new = f"<{r[0]}_msgs/{r[2]}/{r[3]}.hpp>"
        results.append([include_orig, include_new])
        use_orig = f"{r[0]}_msgs::{r[1]}" 
        use_new = f"{r[0]}_msgs::{r[2]}::{r[1]}" 
        results.append([use_orig, use_new])
    return results

msg_includes = msg_replaces(msg_replace)
#print('\n'.join(str(x) for x in msg_includes))

heuristic_re_replaces = [
    [r"(?<=[^a-zA-Z_])node_handle_\.", "this->"],
    [r"(?<=[^a-zA-Z_])node_handle\.", "this->"],
    [r"(?<=[^a-zA-Z_])_node_handle\.", "this->"],
    [r"(?<=[^a-zA-Z_])nh_\.", "this->"],
    [r"(?<=[^a-zA-Z_])nh\.", "this->"],
    [r"(?<=[^a-zA-Z_])_nh\.", "this->"],
    [r"(?<=[^a-zA-Z_])priv_node_handle_\.", "this->"],
    [r"(?<=[^a-zA-Z_])priv_node_handle\.", "this->"],
    [r"(?<=[^a-zA-Z_])_priv_node_handle\.", "this->"],
    [r"(?<=[^a-zA-Z_])private_node_handle_\.", "this->"],
    [r"(?<=[^a-zA-Z_])private_node_handle\.", "this->"],
    [r"(?<=[^a-zA-Z_])_private_node_handle\.", "this->"],
    [r"(?<=[^a-zA-Z_])private_nh_\.", "this->"],
    [r"(?<=[^a-zA-Z_])private_nh\.", "this->"],
    [r"(?<=[^a-zA-Z_])_private_nh\.", "this->"],
    [r"(?<=[^a-zA-Z_])pnh_\.", "this->"],
    [r"(?<=[^a-zA-Z_])pnh\.", "this->"],
    [r"(?<=[^a-zA-Z_])_pnh\.", "this->"],

    [r"(?<=[^a-zA-Z_])node_handle_(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])node_handle(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])_node_handle(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])nh_(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])nh(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])_nh(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])priv_node_handle_(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])priv_node_handle(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])_priv_node_handle(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])private_node_handle_(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])private_node_handle(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])_private_node_handle(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])private_nh_(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])private_nh(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])_private_nh(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])pnh_(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])pnh(?=[^a-zA-Z0-9_])", "this"],
    [r"(?<=[^a-zA-Z_])_pnh(?=[^a-zA-Z0-9_])", "this"],
]

all_deletes = [
    "dynamic_reconfigure",
    "ros::start()",
    "tf/tf.h"
]
all_replaces = msg_includes + [
    ["<actionlib/client/simple_action_client.h>", "<simple_actions/simple_client.hpp>"],
    ["<actionlib/server/simple_action_server.h>", "<simple_actions/simple_server.hpp>"],
    ["actionlib::SimpleActionServer", "simple_actions::SimpleActionServer"],
    ["<message_filters/subscriber.h>", "<message_filters/subscriber.hpp>"],

    ["ros::Publisher", "rclcpp::Publisher<TODO_msgs::msg::>::SharedPtr"],
    ["ros::Subscriber", "rclcpp::Subscription<TODO__msgs::msg::>::SharedPtr"],
    ["ros::ServiceServer", "rclcpp::Service<TODO::srv::>::SharedPtr"],
    [".createTimer(", "->create_wall_timer("],
    [".publish(", "->publish("],
    ["ConstPtr", "ConstSharedPtr"],

    ["<ros/console.h>", "<rclcpp/logging.hpp>"],
    ["<ros/ros.h>", "<rclcpp/rclcpp.hpp>"],
    ["ros::NodeHandle", "rclcpp::Node"],
    ["ros::Rate", "rclcpp::Rate"],
    ["ros::WallTime::now(", "rclcpp::Clock(RCL_SYSTEM_TIME).now("],
    ["ros::WallTime", "rclcpp::Time"],
    ["ros::Duration", "rclcpp::Duration"],
    ["ros::Time::now(", "rclcpp::Clock(RCL_SYSTEM_TIME).now("],
    ["ros::Time()", "rclcpp::Clock(RCL_SYSTEM_TIME).now()"],
    ["ros::Time(0)", "0"],
    [".toSec(", ".seconds("],
    #["ros::Time", "rclcpp::Time"],
    ["ros::spinOnce(", "rclcpp::spin_once("],
    ["ros::spin(", "rclcpp::spin(TODO_NODE"],
    ["ros::ok", "rclcpp::ok"],

    ["tf/LinearMath", "tf2/LinearMath"],
    ["tf/transform_datatypes.h", "tf2/transform_datatypes.h"],
    ["tf/transform_listener.h", "tf2_ros/transform_listener.h"],
    ["<tf2_sensor_msgs/tf2_sensor_msgs.h>", "<tf2_sensor_msgs/tf2_sensor_msgs.hpp>"],
    ["<tf2_geometry_msgs/tf2_geometry_msgs.h>", "<tf2_geometry_msgs/tf2_geometry_msgs.hpp>"],

    ["nav_core/base_local_planner.h", "nav2_core/controller.hpp"],
    ["nav_core::BaseLocalPlanner", "nav2_core::Controller"],
    ["navfn/navfn_ros.h", "nav2_navfn_planner/navfn.hpp"],

    ["pluginlib/class_list_macros.h", "pluginlib/class_list_macros.hpp"],
    ["getParam", "get_parameter"],
    ["getNumSubscribers", "get_subscription_count"],
]

re_replaces = [
    [r"(?<=\.)advertise<", "create_publisher<"],
    [r"(?<=\.)subscribe\(", "create_subscription<TODO::msg::>("],
    [r"(?<=\.)advertiseService\(", "create_service<TODO::srv::>("],
    [r"\.param<(?:[a-zA-Z_][a-zA-Z0-9_]*)(?:::(?:[a-zA-Z_][a-zA-Z0-9_]*))*?>\(", "->declare_parameter("],
    [r"\.param\(", "->declare_parameter("],
    [r"(?:const\s+)?ros::TimerEvent(?:\s+&)?", ""],
    #[r"ros::Timer(?=[^a-zA-Z0-9_])", "rclcpp::WallTimer<rclcpp::VoidCallbackType>::SharedPtr"],
    [r"ros::Timer(?=[^a-zA-Z0-9_])", "rclcpp::TimerBase::SharedPtr"],

    [r'ros::Duration\((.*[^\s]+.*)\)', r'rclcpp::Duration::from_seconds(\1)'],

    [r'ROS_(DEBUG|WARN|ERROR|FATAL|INFO)_STREAM_COND\(', r'RCLCPP_\1_STREAM_EXPRESSION(rclcpp::get_logger("DEFAULT_LOGGER"), '],
    [r'ROS_(DEBUG|WARN|ERROR|FATAL|INFO)_STREAM_NAMED\("((?:\\"|[^"])*?)"', r'RCLCPP_\1_STREAM(rclcpp::get_logger("\2")'],
    [r"(^.*)ROS_(DEBUG|WARN|ERROR|FATAL|INFO)_STREAM_THROTTLE\((.*)", r'\1{\n\1rclcpp::Clock clock(RCL_SYSTEM_TIME);\n\1RCLCPP_\2_STREAM_THROTTLE(rclcpp::get_logger("DEFAULT_LOGGER"), clock, \3\n\1}'],
    [r"ROS_(DEBUG|WARN|ERROR|FATAL|INFO)_STREAM\(", r'RCLCPP_\1_STREAM(rclcpp::get_logger("DEFAULT_LOGGER"), '],
    [r'ROS_(DEBUG|WARN|ERROR|FATAL|INFO)_COND\(', r'RCLCPP_\1_EXPRESSION(rclcpp::get_logger("DEFAULT_LOGGER"), '],
    [r'ROS_(DEBUG|WARN|ERROR|FATAL|INFO)_NAMED\("((?:\\"|[^"])*?)"', r'RCLCPP_\1(rclcpp::get_logger("\2")'],
    [r"(^.*)ROS_(DEBUG|WARN|ERROR|FATAL|INFO)_THROTTLE\((.*)", r'\1{\n\1rclcpp::Clock clock(RCL_SYSTEM_TIME);\n\1RCLCPP_\2_THROTTLE(rclcpp::get_logger("DEFAULT_LOGGER"), clock, \3\n\1}'],
    [r"ROS_(DEBUG|WARN|ERROR|FATAL|INFO)\(", r'RCLCPP_\1(rclcpp::get_logger("DEFAULT_LOGGER"), '],
    [r"ROS_(DEBUG|WARN|ERROR|FATAL|INFO)", r"RCLCPP_\1"],
    [r'ros::init\(([a-zA-Z_][a-zA-Z0-9_]*),\s*([a-zA-Z_][a-zA-Z0-9_]*),\s*(?:"(?:[^"]|\\")*"|[a-zA-Z_][a-zA-Z0-9_]*)\)', r"rclcpp::init(\1, \2)"],

    ["(?<=[^a-zA-Z0-9_])tf::", "tf2::"],
] + heuristic_re_replaces


if __name__ == "__main__":
    infile = sys.argv[1]
    outfile = sys.argv[2]
    out_lines = []
    with open(infile, 'r') as file:
        lines = file.readlines()
        for l in lines:

            # TODO: handle block comments
            if l.strip().startswith('//'):
                continue

            deleted = False
            for deletion in all_deletes:
                if deletion in l:
                    deleted = True
                    break
            if deleted:
                continue
            for old, new in re_replaces:
                l = re.sub(old, new, l)
            for old, new in all_replaces:
                l = l.replace(old, new)

            out_lines.append(l)

    with open(outfile, 'w') as file:
        file.write(''.join(out_lines))

