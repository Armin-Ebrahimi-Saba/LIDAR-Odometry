import rospy
from lio_sam_6axis.msg import cloud_info

def cb(msg):
    print("cloud_corner width:", msg.cloud_corner.width)
    print("cloud_surface width:", msg.cloud_surface.width)
    rospy.signal_shutdown("done")

rospy.init_node("test_node2")
rospy.Subscriber("/lio_sam_6axis/feature/cloud_info", cloud_info, cb)
rospy.spin()
