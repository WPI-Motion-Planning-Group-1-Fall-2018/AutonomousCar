#!/usr/bin/env python

import roslib
import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState


def main():
    rospy.init_node('Set_object_position')

    g_get_state = rospy.ServiceProxy("/gazebo/set_model_state", ModelState, queuesize = 10)

    rospy.wait_for_service("/gazebo/set_model_state")

    try:

        state = g_get_state(model_name="unit_box")



    print(state.pose)


if __name__ == '__main__':
    try:
        main()
except rospy.ROSInterruptException:
pass