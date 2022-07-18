# TurtleBot3 navigate to goal
# ref: https://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

#!/usr/bin/env python

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
import rospy
from tb3_agent import TurtleBotAgent

class GoToGoal(TurtleBotAgent):
    def __init__(self):
        """ initialize GoToGoal object """
        super(GoToGoal, self).__init__()
        self.tol=0.005
    def go_to_goal(self):
        self.set_goal(goal_x=0, goal_y=0)
        self.nav_goal(mode=2)

        if self.goal_dist(self.goal_pose) < self.tol:
            self.stop_bot()
            self.orient_bot()
            print("Reset turtlebot.")

if __name__ == '__main__':
    try:
        rospy.init_node("tb3_reset", anonymous=True)
        agent = GoToGoal()
        agent.go_to_goal()
    except rospy.ROSInterruptException:
        pass
