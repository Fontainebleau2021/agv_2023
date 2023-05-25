#!/usr/bin/env python
import os
from publish_utils import *

if __name__=='__main__':
    rospy.init_node('car_pub_node',anonymous=True)
    ego_pub=rospy.Publisher('ego_car',Marker,queue_size=10)
    model_pub=rospy.Publisher('car_model',Marker,queue_size=10)
    line_pub=rospy.Publisher('line_traj',Marker,queue_size=10)
    bridge=CvBridge()
    rate=rospy.Rate(10)
    frame=0

    while not rospy.is_shutdown():
        
        publish_ego_car(ego_pub)
        publish_car_model(model_pub)
        publish_line_model(line_pub)
        rospy.loginfo('published')
        rate.sleep()
        frame+=1
        frame%=154
