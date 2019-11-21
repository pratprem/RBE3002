from nav_msgs.srv import GetPlan

class Lab3:
    
    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab3')
        #subscribe to target_node
        rospy.Subscriber('/move_base_simple/goal', PoseStamped , self.go_to)

        rospy.sleep(1)

        def run(self):
            """
            Runs the node until Ctrl-C is pressed.
            """
            rospy.spin()

    if __name__ == '__main__':
        planner=PathPlanner()
        planner.run()
