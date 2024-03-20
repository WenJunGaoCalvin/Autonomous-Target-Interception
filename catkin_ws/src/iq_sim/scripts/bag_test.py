import rosbag
from std_msgs.msg import Int32, String

# bag = rosbag.Bag('/home/calvinwen/.ros/test.bag','w')

# try:
#     s = String()
#     s.data = 'foo'

#     i = Int32()
#     i.data = 42

#     bag.write('chatter',s)
#     bag.write('numbers',i)
# finally:
#     bag.close()

bag = rosbag.Bag('/home/calvinwen/.ros/test.bag')
bag = rosbag.Bag('/home/calvinwen/catkin_ws/src/iq_sim/scripts/datalog.bag')
for topic, msg, t in bag.read_messages(topics=['test']):
    print(msg)
bag.close()