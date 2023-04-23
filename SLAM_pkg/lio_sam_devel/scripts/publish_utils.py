#!/usr/bin/env python

from pyexpat import model
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf
import numpy as np
import tf_conversions

FRAME_ID='base_link'

def publish_ego_car(ego_car_pub):
    marker=Marker()
    marker.header.frame_id=FRAME_ID
    marker.header.stamp=rospy.Time.now()

    marker.id=0
    marker.action=Marker.ADD
    marker.lifetime=rospy.Duration()
    marker.type=Marker.LINE_STRIP

    marker.color.r=0.0
    marker.color.g=1.0
    marker.color.b=0.0
    marker.color.a=1.0
    marker.scale.x=0.2

    marker.points=[]
    marker.points.append(Point(10,10,0))
    marker.points.append(Point(0,0,0))
    marker.points.append(Point(10,-10,0))

    ego_car_pub.publish(marker)

def publish_car_model(model_pub):
    mesh_marker=Marker()
    mesh_marker.header.frame_id=FRAME_ID
    #mesh_marker.header.stamp=rospy.Time.now()

    mesh_marker.id=-1
    mesh_marker.lifetime=rospy.Duration()
    mesh_marker.type=Marker.MESH_RESOURCE
    # mesh_marker.mesh_resource="package://kitti_tutorial/Audi R8/Models/Audi R8.dae"
    
    # mesh_marker.mesh_resource="package://lio_sam_devel/car/Car.dae"
    
    mesh_marker.mesh_resource="package://lio_sam/car/Car.dae"
    mesh_marker.pose.position.x=0
    mesh_marker.pose.position.y=0
    mesh_marker.pose.position.z=-0
    q = tf_conversions.transformations.quaternion_from_euler(0,0,np.pi/2)
    mesh_marker.pose.orientation.x=q[0]
    mesh_marker.pose.orientation.y=q[1]
    mesh_marker.pose.orientation.z=q[2]
    mesh_marker.pose.orientation.w=q[3]

    mesh_marker.color.r=1.0
    mesh_marker.color.g=1.0
    mesh_marker.color.b=1.0
    mesh_marker.color.a=1.0

    mesh_marker.scale.x=0.9
    mesh_marker.scale.y=0.9
    mesh_marker.scale.z=0.9

    model_pub.publish(mesh_marker)

