import rospy
from lio_sam_6axis.msg import cloud_info

def cb(msg):
    print("startRingIndex len:", len(msg.startRingIndex))
    print("endRingIndex len:", len(msg.endRingIndex))
    print("pointColInd len:", len(msg.pointColInd))
    print("pointRange len:", len(msg.pointRange))
    rospy.signal_shutdown("done")

rospy.init_node("test_node")
rospy.Subscriber("/lio_sam_6axis/deskew/cloud_info", cloud_info, cb)
rospy.spin()
