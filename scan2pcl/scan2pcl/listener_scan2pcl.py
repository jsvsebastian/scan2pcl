#!/usr/bin/env python
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2 as pc2c
import numpy as np
from tf2_ros import TransformBroadcaster



from sklearn.cluster import DBSCAN

# Laser2PC class converts Scan to PCL and runs clustering-based object detection

class Laser2PC(Node):
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = self.create_publisher(pc2, "LaserPCL", queue_size=1) #self.nombre=rospy.publisher("/nombredelnodo" ,tipo, queue_size=1)
        self.laserSub = self.create_subscription(LaserScan, "scan",self.laserCallback, 10)
        self.detections = np.array([], dtype=np.float32)
        self.detectPub = self.create_publisher(Float32MultiArray, "detections", queue_size=1)
        self.br = TransformBroadcaster(self)

    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data)#cambia a cordenadas cartesianas, debido a q el laser da coordenadas polares
        Xg = pc2c.read_points(cloud_out, skip_nans=True, field_names=("x", "y", "z"))
        cloud_points = np.empty((cloud_out.width, 2))
        a = 0
        for p in Xg:
            #rospy.loginfo(p)
            #rospy.loginfo(p[0])
            cloud_points[a, 0] = p[0] #saving the x axis 
            cloud_points[a, 1] = p[1] # savong the y axis
            #cloud_points[a, 2] = p[2]
            #rospy.loginfo(cloud_points[a,:])
            a = a+1

        #rospy.loginfo(cloud_points)

        # Cluster PCL:
        #scaler = StandardScaler()
        #scaler.fit(cloud_points)
        dbscan = DBSCAN()
        clusters = dbscan.fit_predict(cloud_points) #this createme the vector with the different groups of points

        #Obtain detections
        cluster_ids = np.unique(clusters)
        detections = np.zeros((cluster_ids.size, 2), dtype=np.float32)
      

        for ci in cluster_ids:
            if ci != -1:
                detect = np.mean(cloud_points[clusters == ci, :], axis=0)
                detections[ci, :] = detect
                 
                    t = TransformStamped()

                    # Read message content and assign it to
                    # corresponding tf variables
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'laser'
                    t.child_frame_id = self.turtlename

                    # Turtle only exists in 2D, thus we get x and y translation
                    # coordinates from the message and set the z coordinate to 0
                    t.transform.translation.x = msg.x
                    t.transform.translation.y = msg.y
                    t.transform.translation.z = 0.0

                    # For the same reason, turtle can only rotate around one axis
                    # and this why we set rotation in x and y to 0 and obtain
                    # rotation in z axis from the message
                    q = quaternion_from_euler(0, 0, msg.theta)
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                br.sendTransform((detect[0], detect[1], 0),
                                 tf2.transformations.quaternion_from_euler(0, 0, 0),
                                 self.Time.now(),
                                 "detect_%d" % ci,
                                 "base_scan")
        #

        self.detections = detections
        #self.detectPub.publish(detections.flatten())
        detect_msg = Float32MultiArray()
        detect_msg.data = [x for x in detections.flatten()]
        rospy.loginfo(detect_msg.data)
        detect_msg.layout.dim.append(MultiArrayDimension())
        detect_msg.layout.dim.append(MultiArrayDimension())
        detect_msg.layout.dim[0].label = "height"
        detect_msg.layout.dim[1].label = "width"
        detect_msg.layout.dim[0].size = cluster_ids.size
        detect_msg.layout.dim[1].size = 2
        detect_msg.layout.dim[0].stride = cluster_ids.size * 2
        detect_msg.layout.dim[1].stride = 2
        detect_msg.layout.data_offset = 0
        rospy.loginfo("Detect message")
        self.detectPub.publish(detect_msg)
        rospy.loginfo("Detect message")
        self.pcPub.publish(cloud_out)
        rospy.loginfo("Detection Array Start")
        rospy.loginfo(detections)
        rospy.loginfo("Detection Array EndS")
        #rospy.loginfo(detections)

if __name__ == '__main__':
    rospy.init_node('laser2PC', anonymous=True)
    l2pc = Laser2PC()
    rospy.spin()
