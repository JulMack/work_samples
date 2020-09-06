#!/usr/bin/env python

import rospy
import numpy
import rosbag
import tf
import tf.msg
import math
import geometry_msgs.msg
import sys

# in.bag modifizieren und als out.bag speichern
# Kommentar mit infile_list zur Konfiguration nutzen

def tfFromList(translation_list, rotation_list):
    """
    Creates a geometry_msgs/Transform Object from a translation- and rotation-list object
    """
    transform = geometry_msgs.msg.Transform()
    transform.translation.x, transform.translation.y, transform.translation.z = translation_list
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w = rotation_list
    return transform

def viconIsIdle(transform):
    """
    Returns true if transform-position equals idle (outside of arena) which equals
    translation
        x  1.55
        y -4.52
        z  0.56
    """
    idleX = 1.8
    idleY = -3.3
    if numpy.allclose(transform.translation.x, idleX, 0, 0.1) and \
       numpy.allclose(transform.translation.y, idleY, 0, 0.1):
        return True
    else:
        return False

infile_list = \
[\
#'asym_map_2019-12-06-00-33-00.bag'\
'asym_map_avg_speed_2019-12-06-01-44-11.bag'\
#'asym_map_max_speed_2019-12-06-01-48-00.bag'\
#'sym_map_2019-12-11-17-43-01.bag'\
#'sym_map_avg_speed_2019-12-11-18-32-49.bag'\
#'sym_map_max_speed_2019-12-11-18-52-14.bag'\
]

bool_noslam = False

indir = '/home/user/workspace/ros_data_for_thesis/bagfiles/'
outdir = '/home/user/workspace/ros_data_for_thesis/corrected_bagfiles/'

debug = True

for filename in infile_list:
    inpath = indir + filename
    outpath = outdir + 'slam_' + filename
    print "'%s' > Starting correction" % filename

    bool_laser_done = False
    bool_laser_bfp_tcl_done = False
    bool_laser_bfp_bl_done = False
    bool_laser_bl_flbl_done = False
    bool_laser_flbl_fll_done = False
    bool_world_done = False
    bool_world_gt_done = False
    bool_world_odom_done = False

    # statische Transformationen auslesen
    print "'%s' > Reading static transforms" % filename
    for topic, msg, t in rosbag.Bag(inpath).read_messages():
        # Tree fuer tf(base_footprint -> laser_link) und 
        # tf(top_cover_link -> laser_link) bereitstellen
        if topic == '/tf_static' and not bool_laser_done:
            for trans_stamped in msg.transforms:
                if trans_stamped.header.frame_id == 'rb1_base_c_base_footprint' \
                and trans_stamped.child_frame_id == 'rb1_base_c_top_cover_link' \
                and not bool_laser_bfp_tcl_done:
                    bool_laser_bfp_tcl_done = True
                    tfs_bfp_tcl = trans_stamped
                    tfs_bfp_tcl.header.frame_id = 'base_footprint'
                    tfs_bfp_tcl.child_frame_id = 'top_cover_link'
                elif trans_stamped.header.frame_id == 'rb1_base_c_base_footprint' \
                and trans_stamped.child_frame_id == 'rb1_base_c_base_link' \
                and not bool_laser_bfp_bl_done:
                    bool_laser_bfp_bl_done = True
                    tfs_bfp_bl = trans_stamped
                    tfs_bfp_bl.header.frame_id = 'base_footprint'
                    tfs_bfp_bl.child_frame_id = 'base_link'
                elif trans_stamped.header.frame_id == 'rb1_base_c_base_link' \
                and trans_stamped.child_frame_id == 'rb1_base_c_front_laser_base_link' \
                and not bool_laser_bl_flbl_done:
                    bool_laser_bl_flbl_done = True
                    tfs_bl_flbl = trans_stamped
                    tfs_bl_flbl.header.frame_id = 'base_link'
                    tfs_bl_flbl.child_frame_id = 'laser_base_link'
                elif trans_stamped.header.frame_id == 'rb1_base_c_front_laser_base_link' \
                and trans_stamped.child_frame_id == 'rb1_base_c_front_laser_link' \
                and not bool_laser_flbl_fll_done:
                    bool_laser_flbl_fll_done = True
                    tfs_flbl_fll = trans_stamped
                    tfs_flbl_fll.header.frame_id = 'laser_base_link'
                    tfs_flbl_fll.child_frame_id = 'laser_link'
                if bool_laser_bfp_tcl_done and bool_laser_bfp_bl_done and \
                bool_laser_bl_flbl_done and bool_laser_flbl_fll_done:
                    bool_laser_done = True
        # tf(world -> map) [bzw. falls bool_noslam: tf(world -> odom)] bereitstellen
        elif topic == '/tf':
            for trans_stamped in msg.transforms:
                if trans_stamped.header.frame_id == '/vicon_world' and \
                trans_stamped.child_frame_id == 'vicon/rb1_base_c/rb1_base_c' and \
                not viconIsIdle(trans_stamped.transform) and\
                not bool_world_gt_done:
                    # tf(/vicworld -> vic/R/R) als statische Trafo kopieren: 
                    bool_world_gt_done = True
                    tfs_world_gt_0 = trans_stamped
                    tfs_world_gt_0.header.frame_id = 'world'
                    tfs_world_gt_0.child_frame_id = 'gt'
                if trans_stamped.header.frame_id == 'rb1_base_c_odom' and \
                trans_stamped.child_frame_id == 'rb1_base_c_base_footprint' and \
                not bool_world_odom_done:
                    # tf(/vicworld -> vic/R/R) als statische Trafo kopieren: 
                    bool_world_odom_done = True
                    tfs_odom_bfp_0 = trans_stamped
                    tfs_odom_bfp_0.header.frame_id = 'odom'
                    tfs_odom_bfp_0.child_frame_id = 'base_footprint'
                if bool_world_gt_done and bool_world_odom_done:
                    bool_world_done = True
        elif bool_laser_done and bool_world_done:
            break

    if not (bool_laser_done and bool_world_done):
        if debug: 
            print 'DEBUG > Error. Transforms missing' 
        sys.exit()

    tfs_world_offset = geometry_msgs.msg.TransformStamped()

    # set timestamp = 0 due to issues
    tfs_bfp_tcl.header.stamp = rospy.Time()
    tfs_bfp_bl.header.stamp = rospy.Time()
    tfs_bl_flbl.header.stamp = rospy.Time()
    tfs_flbl_fll.header.stamp = rospy.Time()
    tfs_world_gt_0.header.stamp = rospy.Time()
    tfs_odom_bfp_0.header.stamp = rospy.Time()
    tfs_world_offset.header.stamp = rospy.Time()

    # chain static transforms
    print "'%s' > Chaining static transforms" % filename
    static_transformer = tf.Transformer(False, rospy.Duration(10))

    # feed transformer part 1
    static_transformer.setTransform(tfs_world_gt_0)
    static_transformer.setTransform(tfs_odom_bfp_0)

    # cast transform for starting position correction (offset)
    #     T_Off,Odom = I -> T_W,Off o T_Off,bfp,0 = T_W,gt,0
    # <=> T_W,Off = T_W,gt,0 o T_Off,bfp,0^-1

    trans_list_world_gt, rot_list_world_gt = static_transformer.lookupTransform\
        ('world', 'gt', rospy.Time(0))
    trans_mat_world_gt = tf.transformations.translation_matrix(trans_list_world_gt)
    rot_mat_world_gt = tf.transformations.quaternion_matrix(rot_list_world_gt)
    mat_world_gt = numpy.dot(trans_mat_world_gt, rot_mat_world_gt)
    
    trans_list_offset_bfp, rot_list_offset_bfp = static_transformer.lookupTransform\
        ('odom', 'base_footprint', rospy.Time(0))
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
    
    # feed transformer part 2
    static_transformer.setTransform(tfs_bfp_tcl)
    static_transformer.setTransform(tfs_bfp_bl)
    static_transformer.setTransform(tfs_bl_flbl)
    static_transformer.setTransform(tfs_flbl_fll)

    # cast other  chained static transform-objects
    # base_link -> laser_link
    tfs_bl_ll = geometry_msgs.msg.TransformStamped()
    trans_list_bl_ll, list_rot_bl_ll = static_transformer.lookupTransform\
        ('base_link', 'laser_link', rospy.Time(0))
    tfs_bl_ll.header.frame_id = 'base_link'
    tfs_bl_ll.child_frame_id = 'laser_link'
    tfs_bl_ll.transform = tfFromList(trans_list_bl_ll, list_rot_bl_ll)

    # gt -> gt_laser_link
    tfs_gt_gtll = geometry_msgs.msg.TransformStamped()
    list_trans_gt_gtll, list_rot_gt_gtll = static_transformer.lookupTransform\
        ('top_cover_link', 'laser_link', rospy.Time(0))
    tfs_gt_gtll.header.frame_id = 'gt'
    tfs_gt_gtll.child_frame_id = 'gt_laser_link'
    tfs_gt_gtll.transform = tfFromList(list_trans_gt_gtll, list_rot_gt_gtll)

    # base_footprint -> laser_link
    tfs_bfp_ll = geometry_msgs.msg.TransformStamped()
    list_trans_bfp_ll, list_rot_bfp_ll = static_transformer.lookupTransform\
        ('base_footprint', 'laser_link', rospy.Time(0))
    tfs_bfp_ll.header.frame_id = 'base_footprint'
    tfs_bfp_ll.child_frame_id = 'laser_link'
    tfs_bfp_ll.transform = tfFromList(list_trans_bfp_ll, list_rot_bfp_ll)


    # out.bag Objekt zum schreiben definieren
    print "'%s' > Writing bagfile" % filename
    with rosbag.Bag(outpath, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inpath).read_messages():
            # Scan-Msgs fuer Roboter und Ground Truth bereitstellen
            if topic == '/rb1_base_c/scan':
                msg.header.frame_id = 'laser_link'
                topic = '/robot/scan'
                outbag.write(topic, msg, t)
                msg.header.frame_id = 'gt_laser_link' 
                topic = '/gt/scan'
                outbag.write(topic, msg, t)
            elif topic == '/tf':
                for trans_stamped in msg.transforms:
                    # world -> odom (bzw. world -> map) und world -> gt bereitstellen
                    if trans_stamped.header.frame_id == '/vicon_world' and \
                    trans_stamped.child_frame_id == 'vicon/rb1_base_c/rb1_base_c' and \
                    not viconIsIdle(trans_stamped.transform):
                        trans_stamped.header.frame_id = 'world'
                        trans_stamped.child_frame_id = 'gt'
                        newmsg = tf.msg.tfMessage()
                        newmsg.transforms.append(trans_stamped)
                        tfs_gt_gtll.header.stamp = trans_stamped.header.stamp + rospy.Duration(0, 500)
                        newmsg.transforms.append(tfs_gt_gtll)
                        outbag.write(topic, newmsg, t)
                    elif trans_stamped.header.frame_id == 'rb1_base_c_odom' and \
                    trans_stamped.child_frame_id == 'rb1_base_c_base_footprint':
                        #TODO
                        if bool_noslam:
                            trans_stamped.header.frame_id = 'offset'
                        else:
                            trans_stamped.header.frame_id = 'odom'
                        trans_stamped.child_frame_id = 'base_footprint'
                        newmsg = tf.msg.tfMessage()
                        newmsg.transforms.append(trans_stamped)
                        tfs_bfp_bl.header.stamp = trans_stamped.header.stamp + rospy.Duration(0, 500)
                        newmsg.transforms.append(tfs_bfp_bl)
                        tfs_bl_ll.header.stamp = trans_stamped.header.stamp + rospy.Duration(0, 1000)
                        newmsg.transforms.append(tfs_bl_ll)
                        tfs_world_offset.header.stamp = trans_stamped.header.stamp + rospy.Duration(0, 1500)
                        newmsg.transforms.append(tfs_world_offset)
                        outbag.write(topic, newmsg, t)
    print "'%s' > Success." % filename