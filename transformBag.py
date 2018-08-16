#!/usr/bin/python2.7
import rosbag
import csv

import numpy as np

import tf

# bagName='2018-06-28-22-09-33'
bagName = '2018-07-05-12-26-17' # first video simulation
# bagName = '2018-08-15-23-19-25'
# bagName = '2018-08-15-23-38-28'
# bagName = '2018-06-28-22-09-33partial3'
# bagName = '2018-08-16-01-15-41'
# bagName = '2018-08-16-02-29-54'

bagFile = '/home/lukas/bagfiles/Raw/' + bagName + '.bag'

# # Bafile info
# import yaml
# info_dict = yaml.load(rosbag.Bag(bagFile, 'r')._get_yaml_info())
# print('info dict', info_dict)
# import pdb; pdb.set_trace() # Break point pythno
# bag = rosbag.Bag(bagFile)

print('')
print("Let's do the unbagging")
print('')

N_link = 7

recieved_tf = [0 for ii in range(N_link)]   # Transfomr cehcker
tf_links = [0 for ii in range(N_link)]   # Transforms between links
dist_links = np.zeros((3, N_link))   # Link distance in world frame

print('Opening file .....')
with rosbag.Bag(bagFile, 'r') as bag:
    print('.... and file is open.')
    with open('MeasurementClean/' + bagName + ".csv", "w") as f:
        writer = csv.writer(f, delimiter=';')

        writer.writerow(["t"] + N_link*["x", "y", "z"])
        it_write = 0
        # for topic, msg, t in bag.read_messages(topics=['/lwr/ee_pose',]):
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            # print('t:', t, '       msg', msg)
            # import pdb; pdb.set_trace() # Break point python
            
            for ii in range(len(msg.transforms)):
                # print('anther ', ii)
                # print('kids frame', msg.transforms[ii].child_frame_id)
                # print('parental frame', msg.transforms[ii].header.frame_id)
                
                # Check child frame id
                if((len(msg.transforms[ii].child_frame_id) == 10 and
                   msg.transforms[ii].child_frame_id[:4] == 'lwr_' and
                   msg.transforms[ii].child_frame_id[5:] == '_link')):
                   # or
                   # msg.transforms[ii].child_frame_id == 'lwr_base_link'):
                    
                    # check parent frame id
                    # if msg.transforms[ii].child_frame_id == 'lwr_base_link':
                        # nn = 0
                    # else:
                    nn = int(msg.transforms[ii].child_frame_id[4])-1
                    # print('nn', nn)
                    
                    # if(nn==0 and msg.transforms[ii].header.frame_id == 'world'):
                        # import pdb; pdb.set_trace() # Break point python                    
                        # tf_links[nn] = msg.transforms[ii].transform  #
                        # recieved_tf[nn] = 1
                    if(nn==0 and msg.transforms[ii].header.frame_id == 'lwr_base_link'):
                        # import pdb; pdb.set_trace() # Break point python                    
                        tf_links[nn] = msg.transforms[ii].transform
                        recieved_tf[nn] = 1
                    elif(msg.transforms[ii].header.frame_id == 'lwr_{}_link'.format(nn)):
                        tf_links[nn] = msg.transforms[ii].transform
                        recieved_tf[nn] = 1
                        
                    # import pdb; pdb.set_trace() # Break point python                    
                    if nn==N_link-1 and sum(recieved_tf)==N_link:  # Write row every time new end effector value
                        # import pdb; pdb.set_trace() # Break point python                    
                        # and sum(tf_links) ==N_link
                        # Distance in world frame
                        
                        dist_world = np.zeros((3)) # Maybe add end effector pos
                        
                        dist_quat = np.hstack((dist_world, 0))
                        # print('dist', dist_quat)

                        dist_links = np.zeros((3, N_link))
                        
                        for jj in range(N_link-1, -1, -1):
                            # print('jj', jj)
                            dist = np.array([tf_links[jj].translation.x,
                                             tf_links[jj].translation.y,
                                             tf_links[jj].translation.z])
                            quat = np.array([tf_links[jj].rotation.x,
                                             tf_links[jj].rotation.y,
                                             tf_links[jj].rotation.z,
                                             tf_links[jj].rotation.w])
                            # Calcluate conjucate (equivalent to inverse)
                            quat_conj = tf.transformations.quaternion_inverse(quat)
                            
                            dist_quat = (np.hstack((dist, 0)) +
                                         tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply( quat, dist_quat ), quat_conj) )
                            
                            for kk in range(N_link-1, jj, -1):
                                dist_links_q = np.hstack((dist_links[:, kk], 0 ))
                                # dist_links_q = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(quat_conj, dist_links_q), quat)nn
                                dist_links_q = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(quat, dist_links_q), quat_conj)
                                dist_links[:, kk] = dist_links_q[0:3]
                                # print('kk', kk)

                            # import pdb; pdb.set_trace() # Break point python
                            # print('dist links', dist_links)
                            dist_links[:, jj] = dist
                            
                        dist_world = dist_quat[1:]
                        
                        # writer.writerow(["{}.{}".format(t.secs, t.nsecs),
                                         # msg.transforms[ii].transform.translation.x,
                                         # msg.transforms[ii].transform.translation.y,
                                         # msg.transforms[ii].transform.translation.
                                         
                        # writer.writerow(["{}.{}".format(t.secs, t.nsecs),
                                         # dist_world[0],
                                         # dist_world[1],
                                         # dist_world[2] ] )
                        lineToWrite = ["{}.{}".format(t.secs, t.nsecs)]
                        for ll in range(N_link):
                            lineToWrite = lineToWrite + [dist_links[0, ll],
                                                         dist_links[1, ll],
                                                         dist_links[2, ll]]
                                            
                        writer.writerow(lineToWrite)

                        it_write += 1
                        print('DATASET #{}'.format(it_write))
                        # if it_write > 10:
                            # break
                        # print(' writer strucks again')
                        # print(' dist world:', dist_world)

            # ii+1=l
                    # if ii>100: break
                # if ii>100: break
            # if ii>100: break


            
print('n_transforms', it_write)
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
    
print('')
print('Finished the bag fun')
print('')


bag.close()
 
