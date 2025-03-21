import sagepy
import rospy

rospy.init_node('simple_action_talker', anonymous=True)

sagepy.rotate(theta=1.57, rad=-0.3)
# sagepy.move_by_distance(distance=.2, speed=-0.2)
