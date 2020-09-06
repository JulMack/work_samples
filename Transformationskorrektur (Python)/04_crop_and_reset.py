#!/usr/bin/env python

import rospy
import numpy
import rosbag
import tf
import tf.msg
import math
import geometry_msgs.msg

inDir = '/home/user/workspace/ros_data_for_thesis/corrected_bagfiles/'
# outDir = '/home/user/workspace/ros_data_for_thesis/datasets/00_bagfiles/odometry/odom_'
outDir = '/home/user/workspace/ros_data_for_thesis/datasets/00_bagfiles/'
bool_noslam_inbag = False
bool_noslam_outbag = False

targets = (
    # (pathIn, pathOut, needsCrop, tStart, tEnd)
    (inDir + 'slam_02_asym_avg.bag',    outDir + '01_asym_multilap_avg.bag', False, 0.00, 0.00),
    (inDir + 'slam_05_sym_avg.bag',     outDir + '02_sym_multilap_avg.bag', False, 0.00, 0.00),
    (inDir + 'slam_02_asym_avg_c.bag',  outDir + '03_asym_singlelap_slow.bag', False, 0.00, 0.00),
    (inDir + 'slam_03_asym_fast_c.bag', outDir + '04_asym_singlelap_fast.bag', False, 0.00, 0.00),
    (inDir + 'slam_04_sym_slow_c.bag',  outDir + '05_sym_singlelap_slow.bag', True, 1576082847.0, 1576082997.0),
    (inDir + 'slam_06_sym_fast_b.bag',  outDir + '06_sym_singlelap_fast.bag', True, 1576086801.0, 1576086813.0),
    (inDir + 'slam_01_asym_slow_c.bag', outDir + '07_asym_short_slow.bag', False, 0.00, 0.00),
    (inDir + 'slam_02_asym_avg_d.bag',  outDir + '08_asym_short_avg.bag', False, 0.00, 0.00),
    (inDir + 'slam_03_asym_fast_d.bag', outDir + '09_asym_short_fast.bag', False, 0.00, 0.00),
    (inDir + 'slam_04_sym_slow_c.bag',  outDir + '10_sym_short_slow.bag', True, 1576083004.0, 1576083038.0),
    (inDir + 'slam_05_sym_avg_b.bag',   outDir + '11_sym_short_avg.bag', True,  1576085685.0, 1576085706.0),
    (inDir + 'slam_06_sym_fast_a.bag',  outDir + '12_sym_short_fast.bag', False, 0.00, 0.00)
)

for pathIn, pathOut, needsCrop, tStart, tEnd in targets:
    # Print current file
    print 'fetching: %s \n casting:  %s' % (pathIn, pathOut)

    bool_world_gt_done = False
    bool_world_odom_done = False

    inbag = rosbag.Bag(pathIn)
    
    # Checken, ob es sich um einen Crop handelt oder nicht
    if not needsCrop:
        tStart = inbag.get_start_time()
        tEnd = inbag.get_end_time()

    # Daten fuer neue, erste Offset-Trafo auslesen:
    for topic, msg, t in inbag.read_messages():
        if topic == '/tf':
            for trans_stamped in msg.transforms:
                # Es wird nur die erste Trafo nach Startzeit beachtet
                if t.to_sec() > tStart:
                    if trans_stamped.header.frame_id == 'world' and \
                            trans_stamped.child_frame_id == 'gt':
                        # WORLD -> GT Trafo auslesen
                        bool_world_gt_done = True
                        tfs_world_gt_1 = trans_stamped
                        tfs_world_gt_1.header.frame_id = 'world'
                        tfs_world_gt_1.child_frame_id = 'gt'
                    # OFFSET -> BFP Trafo auslesen (noslam -> es gibt keinen odom-frame)
                    elif bool_noslam_inbag and \
                            trans_stamped.header.frame_id == 'offset' and \
                            trans_stamped.child_frame_id == 'base_footprint':
                        bool_world_odom_done = True
                        tfs_odom_bfp_1 = trans_stamped
                        tfs_odom_bfp_1.header.frame_id = 'odom'
                        tfs_odom_bfp_1.child_frame_id = 'base_footprint'
                    # ODOM -> BFP Trafo auslesen (slam -> es gibt keine direkte
                    # Verbindung zwischen offset und odom-frame)
                    elif not bool_noslam_inbag and \
                            trans_stamped.header.frame_id == 'odom' and \
                            trans_stamped.child_frame_id == 'base_footprint':
                        bool_world_odom_done = True
                        tfs_odom_bfp_1 = trans_stamped
                        tfs_odom_bfp_1.header.frame_id = 'odom'
                        tfs_odom_bfp_1.child_frame_id = 'base_footprint'
            if bool_world_gt_done and bool_world_odom_done:
                # Letzte gemeinsame Zeit als neue Startzeit uebernehmen
                tStart = t.to_sec()
                break

    # Offset-Trafo als Objekt erzeugen
    tfs_world_offset = geometry_msgs.msg.TransformStamped()

    tfs_odom_bfp_1.header.stamp = rospy.Time()
    tfs_world_gt_1.header.stamp = rospy.Time()
    tfs_world_offset.header.stamp = rospy.Time()

    static_transformer = tf.Transformer(False, rospy.Duration(10))
    static_transformer.setTransform(tfs_world_gt_1)
    static_transformer.setTransform(tfs_odom_bfp_1)

    trans_list_world_gt, rot_list_world_gt = static_transformer.lookupTransform(
        'world', 'gt', rospy.Time(0))
    trans_mat_world_gt = tf.transformations.translation_matrix(trans_list_world_gt)
    rot_mat_world_gt = tf.transformations.quaternion_matrix(rot_list_world_gt)
    mat_world_gt = numpy.dot(trans_mat_world_gt, rot_mat_world_gt)

    trans_list_offset_bfp, rot_list_offset_bfp = static_transformer.lookupTransform(
        'odom', 'base_footprint', rospy.Time(0))
    trans_mat_offset_bfp = tf.transformations.translation_matrix(trans_list_offset_bfp)
    rot_mat_offset_bfp = tf.transformations.quaternion_matrix(rot_list_offset_bfp)
    mat_offset_bfp = numpy.dot(trans_mat_offset_bfp, rot_mat_offset_bfp)
    mat_offset_bfp_inverse = tf.transformations.inverse_matrix(mat_offset_bfp)
    mat_world_offset = numpy.dot(mat_world_gt, mat_offset_bfp_inverse)
    
    trans_list_world_offset = tf.transformations.translation_from_matrix(mat_world_offset)
    rot_list_world_offset = tf.transformations.quaternion_from_matrix(mat_world_offset)
    tfs_world_offset.transform.translation.x, \
    tfs_world_offset.transform.translation.y, \
    tfs_world_offset.transform.translation.z = trans_list_world_offset
    tfs_world_offset.transform.rotation.x, \
    tfs_world_offset.transform.rotation.y, \
    tfs_world_offset.transform.rotation.z, \
    tfs_world_offset.transform.rotation.w  = rot_list_world_offset
    tfs_world_offset.header.frame_id = 'world'
    tfs_world_offset.child_frame_id = 'offset'

    tfs_world_gt_1.header.frame_id = 'world'
    tfs_world_gt_1.child_frame_id = 'start'

    with rosbag.Bag(pathOut, 'w') as outbag:
        for topic, msg, t in inbag.read_messages():
            if t.to_sec() >= tEnd:
                break
            elif t.to_sec() >= tStart: 
                # WORLD -> OFFSET mit der neuen Trafo ueberschreiben
                if topic == '/tf':
                    newmsg = tf.msg.tfMessage()
                    for trans_stamped in msg.transforms:
                        if trans_stamped.header.frame_id == 'world' and \
                                trans_stamped.child_frame_id == 'offset':
                            trans_stamped.transform = tfs_world_offset.transform
                            tfs_world_gt_1.header.stamp = trans_stamped.header.stamp + rospy.Duration(0, 500)
                            newmsg.transforms.append(tfs_world_gt_1)
                        elif bool_noslam_outbag and not bool_noslam_inbag and \
                                trans_stamped.header.frame_id == 'odom' and \
                                trans_stamped.child_frame_id == 'base_footprint':
                            trans_stamped.header.frame_id = 'offset'
                            trans_stamped.child_frame_id = 'base_footprint'
                        newmsg.transforms.append(trans_stamped)
                    outbag.write(topic, newmsg, t)
                else:
                    outbag.write(topic, msg, t)

    print 'Success.\n'