#!/usr/bin/env python3

import rospy
from panda_dummy.msg import PeginholeRequest, PeginholeResult

class PandaDummy:
    grasping_time = 50 # 5sec

    def __init__(self):
        self.grasp_pose = None
        self.tool_pose = None
        self.pub_result = rospy.Publisher('peginhole_result', PeginholeResult, queue_size=1)
        rospy.Subscriber("peginhole_request", PeginholeRequest, self.peginhole_request_callback)
        rospy.init_node('panda_dummy')
        self.rate = rospy.Rate(10)

        # result: 0-idle 1-doing 2-Done 3~-errorcode(TBD)
        self.peginhole_state = 0 
        self.cnt = 0

    def peginhole_request_callback(self, data):
        """ Callback function for peginhole_request topic
        """
        if self.peginhole_state == 0:
            self.peginhole_state = 1
            self.O_T_grasp = data.O_T_grasp
            self.grasp_T_tool = data.grasp_T_tool
            rospy.loginfo("Request received!")
        else:
            rospy.loginfo("Do not request while running!")
    
    def peginhole_publish(self, is_done, errorcode=0):
        """ Publish function of peginhole_result topic
        """
        result = PeginholeResult()
        result.state = self.peginhole_state
        result.is_done = is_done
        result.errorcode = 0
        self.pub_result.publish(result)

    def peginhole_state_manager(self):
        """ State Machine manager of peg-in-hole process
        """
        if self.peginhole_state == 0:
            # IDLE
            pass

        elif self.peginhole_state == 1:
            # DOING
            rospy.loginfo("Running...")
            self.cnt += 1
            self.peginhole_publish(is_done=False)
            #transition
            if self.cnt >= 50: #after 5 sec
                self.cnt = 0
                self.peginhole_state = 2

        elif self.peginhole_state == 2:
            # DONE
            rospy.loginfo("Done!!")
            self.peginhole_publish(is_done=True)
            #transition
            self.peginhole_state = 0

    def run(self):
        """ While loop
        """
        while not rospy.is_shutdown():
            self.peginhole_state_manager()
            self.rate.sleep()


if __name__ == '__main__':
    robot = PandaDummy()
    robot.run()