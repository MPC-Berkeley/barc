import rosbag

from std_msgs.msg import Int32, String
   3 
   4 bag = rosbag.Bag('test.bag', 'w')
   5 
   6 try:
   7     str = String()
   8     str.data = 'foo'
   9 
  10     i = Int32()
  11     i.data = 42
  12 
  13     bag.write('chatter', str)
  bag.write('numbers', i)
  finally:
  bag.close()