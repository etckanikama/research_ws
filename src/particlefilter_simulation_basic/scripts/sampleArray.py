# #!/usr/bin/env python3
# import roslib; roslib.load_manifest('visualization_marker_tutorials')
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import PoseArray
# import rospy

# topic = 'test_poses'
# publisher = rospy.Publisher(topic, PoseArray)
 
# rospy.init_node('posearray')
 
# while not rospy.is_shutdown():
 
#     ps = PoseArray()
#     ps.header.frame_id = "/base_link"
#     ps.header.stamp = rospy.Time.now()
    
#     pose = Pose()
#     pose.position.x = 2
#     pose.position.y = 2
#     pose.position.z = 0
# 00023    pose.orientation.x = 0
# 00024    pose.orientation.y = 0
# 00025    pose.orientation.z = .717
# 00026    pose.orientation.w = .717
# 00027 
# 00028    ps.poses.append( pose )
# 00029 
# 00030    pose = Pose()
# 00031    pose.position.x = 1
# 00032    pose.position.y = 1
# 00033    pose.position.z = 0
# 00034    pose.orientation.x = 0
# 00035    pose.orientation.y = 0
# 00036    pose.orientation.z = 0
# 00037    pose.orientation.w = 1
# 00038 
# 00039    ps.poses.append( pose )
# 00040 
# 00041    publisher.publish( ps )
#     rospy.sleep(0.1)
