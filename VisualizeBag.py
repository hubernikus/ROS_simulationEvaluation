#!/usr/bin/python2.7
import rosbag
import csv

bagName='2018-06-28-22-09-33.bag'
bagFile = '/home/lukas/bagfiles/Raw/'+bagName


# # Bafile info
# import yaml
# info_dict = yaml.load(rosbag.Bag(bagFile, 'r')._get_yaml_info())
# print('info dict', info_dict)
# import pdb; pdb.set_trace() # Break point pythno
# bag = rosbag.Bag(bagFile)

print('And our beloved rosbag is open')
print('')
print('')


with rosbag.Bag(bagFile, 'r') as bag:
    with open(bagName+".csv", "w") as f:
        writer = csv.writer(f, delimiter=';')
        # writer.writerow([1, 2, 3])
        # writer.writerow(["a", "b", "c"])

        writer.writerow(["t", "x", "y", "z"])

        ii = 0
        for topic, msg, t in bag.read_messages(topics=['/lwr/ee_pose']):
            # print('t:', t, '       msg', msg)
            # import pdb; pdb.set_trace() # Break point python
            writer.writerow(["{}.{}".format(t.secs, t.nsecs),
                             msg.position.x,
                             msg.position.y,
                             msg.position.z])
            # ii+=1
            # if ii>100: break
            

# for topic, msgs, t in bag.read_messages(topics=['/attr/pose']):
    # print('msg:',msgs)

# topic_name = '/attr/pose'
# for topic, msg, t in bag.read_messages(topics=topic_name):
    # print('msg:', msg)
    # print('topic:', topic)
    # print('t:', t)

# topics = bag.get_type_and_topic_info()[1].keys()
# types = []

# for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
    # types.append(bag.get_type_and_topic_info()[1].values()[i][0])

print('Finished the bag fun')
bag.close()
