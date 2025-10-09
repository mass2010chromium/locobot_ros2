from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration

from klampt.math import so3
import math




def main() -> None:
    rclpy.init()
    nav = BasicNavigator()
    print("Cancelling nav tasks")
    nav.cancelTask()
    input("Enter to go")
    
    ####this part isn't completely necessary. however it's useful when running the script more than once
    #init_pose = PoseStamped() #default (x, y, z) pos = 0.0
    #init_pose.header.frame_id = 'map'
    #init_pose.header.stamp = nav.get_clock().now().to_msg()
    #init_pose.pose.position.x = 0.0
    #init_pose.pose.position.y = 0.0
    #init_pose.pose.position.z = 0.0
    #init_pose.pose.orientation.w = 1.0 #other two quaternion values set to 0 by default
    #init_pose.pose.orientation.z = 0.0
    #nav.setInitialPose(init_pose)
    ####

    r = so3.from_axis_angle(([0, 0, 1], 0*(2*math.pi)))
    w, x, y, z = so3.quaternion(r)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = -0.2
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = w
    goal_pose.pose.orientation.x = x
    goal_pose.pose.orientation.y = y
    goal_pose.pose.orientation.z = z



    #path = nav.getPath(init_pose, goal_pose) #to see if a valid path exists
    nav.goToPose(goal_pose)

    i = 0
    start_time = nav.get_clock().now()
    while not nav.isTaskComplete():
        i += 1
        feedback = nav.getFeedback()
        if feedback:
            print('ETA: ' + '{:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds/1e9)
                  + ' secs')
            
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds = 90):
                print("Took too long, cancelling task...")
                nav.cancelTask()
    end_time = nav.get_clock().now()
    duration = end_time - start_time
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'Goal succeeded in {duration.nanoseconds * 1e-9:.2f} seconds')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has invalid return status!')

    #nav.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
