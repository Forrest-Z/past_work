#!/usr/bin/env python


import rospy
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import *
import os
import sys
import numpy as np
# import tensorflow as tf
import argparse
import pandas as pd
from sklearn.neighbors import NearestNeighbors , KDTree
import math
import errno
import multiprocessing as multiproc
#import glog as loggersys.argv[1:]
from copy import deepcopy
from functools import partial
import time



parse = argparse.ArgumentParser()
parse.add_argument('--k_start', type=int, default=20,
                       help="KNN cluster k range start point")
parse.add_argument('--k_end', type=int, default=100,
                       help="KNN cluster k range end point")
parse.add_argument('--k_step', type=int, default=10,
                       help="KNN cluster k range step")
parse.add_argument('--bin_core_num', type=int, default=10, help="Parallel process file Pool core num")
args = parse.parse_args()


def feature_array_to_pointcloud2(array):
    msg=pc2.create_cloud(header=None,fields= [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.FLOAT32, 1),
        PointField('g', 16, PointField.FLOAT32, 1),
        PointField('b', 20, PointField.FLOAT32, 1),
        PointField('mf1', 24, PointField.FLOAT32, 1),
        PointField('mf2', 28, PointField.FLOAT32, 1),
        PointField('mf3', 32, PointField.FLOAT32, 1),
        PointField('mf4', 36, PointField.FLOAT32, 1),
        PointField('mf5', 40, PointField.FLOAT32, 1),
        PointField('mf6', 44, PointField.FLOAT32, 1),
        PointField('mf7', 48, PointField.FLOAT32, 1)],points=array)

    return msg


def calculate_features(pointcloud, nbrs_index, eigens_, vectors_):
    ### calculate handcraft feature with eigens and statistics data

    # print(nbrs_index.shape,"\n",eigens_,"\n",vectors_,"\n")

    # features using eigens
    eig3d = eigens_[:3]
    eig2d = eigens_[3:5]

    # 3d
    C_ = eig3d[2] / (eig3d.sum())
    O_ = np.power((eig3d.prod() / np.power(eig3d.sum(), 3)), 1.0 / 3)
    L_ = (eig3d[0] - eig3d[1]) / eig3d[0]
    E_ = -((eig3d / eig3d.sum()) * np.log(eig3d / eig3d.sum())).sum()
    #P_ = (eig3d[1] - eig3d[2]) / eig3d[0]
    #S_ = eig3d[2] / eig3d[0]
    #A_ = (eig3d[0] - eig3d[2]) / eig3d[0]
    #X_ = eig3d.sum()
    D_ = 3 * nbrs_index.shape[0] / (4 * math.pi * eig3d.prod())
    # 2d
    S_2 = eig2d.sum()
    L_2 = eig2d[1] / eig2d[0]
    # features using statistics data
    neighborhood = pointcloud[nbrs_index]
    nbr_dz = neighborhood[:, 2] - neighborhood[:, 2].min()
    dZ_ = nbr_dz.max()
    vZ_ = np.var(nbr_dz)
    V_ = vectors_[2][2]

    features = np.asarray([C_, O_, L_, E_,  D_, S_2, L_2, dZ_, vZ_, V_])#([C_,O_,L_,E_,D_,S_2,L_2,dZ_,vZ_,V_])
    return features

def calculate_entropy(eigen):
    L_ = (eigen[0] - eigen[1]) / eigen[0]
    P_ = (eigen[1] - eigen[2]) / eigen[0]
    S_ = eigen[2] / eigen[0]
    Entropy = -L_*np.log(L_)-P_*np.log(P_)-S_*np.log(S_)
    return Entropy

def calculate_entropy_array(eigen):
    L_ = (eigen[:,0] - eigen[:,1]) / eigen[:,0]
    P_ = (eigen[:,1] - eigen[:,2]) / eigen[:,0]
    S_ = eigen[:,2] / eigen[:,0]
    Entropy = -L_*np.log(L_)-P_*np.log(P_)-S_*np.log(S_)
    return Entropy

def covariation_eigenvalue(neighborhood_index, args):
    ### calculate covariation and eigenvalue of 3D and 2D
    # prepare neighborhood
    neighborhoods = args.pointcloud[neighborhood_index]
    # print("neighborhoods.shape ",neighborhoods.shape)

    # 3D cov and eigen by matrix

    Ex = np.average(neighborhoods, axis=1)
    Ex = np.reshape(np.tile(Ex,[neighborhoods.shape[1]]), neighborhoods.shape)
    P = neighborhoods-Ex
    cov_ = np.matmul(P.transpose((0,2,1)),P)/(neighborhoods.shape[1]-1)
    eigen_, vec_ = np.linalg.eig(cov_)   # eigen_ 特征值　 # vec_　　特征向量
    indices = np.argsort(eigen_)
    indices = indices[:,::-1]
    pcs_num_ = eigen_.shape[0]
    indx = np.reshape(np.arange(pcs_num_), [-1, 1])
    eig_ind = indices + indx*3
    vec_ind = np.reshape(eig_ind*3, [-1,1]) + np.full((pcs_num_*3,3), [0,1,2])
    vec_ind = np.reshape(vec_ind, [-1,3,3])
    eigen3d_ = np.take(eigen_, eig_ind)
    vectors_ = np.take(vec_, vec_ind)
    entropy_ = calculate_entropy_array(eigen3d_)

    # 2D cov and eigen
    cov2d_ = cov_[:,:2,:2]
    eigen2d, vec_2d = np.linalg.eig(cov2d_)
    indices = np.argsort(eigen2d)
    indices = indices[:, ::-1]
    pcs_num_ = eigen2d.shape[0]
    indx = np.reshape(np.arange(pcs_num_), [-1, 1])
    eig_ind = indices + indx * 2
    eigen2d_ = np.take(eigen2d, eig_ind)

    eigens_ = np.append(eigen3d_,eigen2d_,axis=1)

    return cov_, entropy_, eigens_, vectors_

def build_neighbors(k, args):
    ### using KNN NearestNeighbors cluster according k
    times = time.time()
    nbrs = NearestNeighbors(n_neighbors=k).fit(args.pointcloud)
    distances, indices = nbrs.kneighbors(args.pointcloud)
    covs, entropy, eigens_, vectors_ = covariation_eigenvalue(indices, args)
    print(covs.shape, "\n",len(entropy),"\n",eigens_.shape, "\n",vectors_.shape,"\n",)
    neighbors = {}
    neighbors['k']=k
    neighbors['indices']=indices
    neighbors['covs']=covs
    neighbors['entropy']=entropy
    neighbors['eigens_'] = eigens_
    neighbors['vectors_'] = vectors_
    #logger.info("KNN:{}".format(k))
    timed = time.time()
    #print('cluster:{}'.format(timed-times))
    return neighbors

def generate_feature(pointcloud, args):
    ### Parallel process pointcloud files
    # load pointcloud file
    args.pointcloud = pointcloud

    time0 = time.time()

    # prepare KNN cluster number k
    cluster_number = []
    for ind in range(((args.k_end-args.k_start)//args.k_step)+1):
        cluster_number.append(args.k_start + ind*args.k_step)

    #logger.info("Processing pointcloud file:{}".format(pointcloud_file))
    #k_nbrs = []
    #for k in cluster_number:
    #    k_nbr = build_neighbors(k, args)
    #    k_nbrs.append(k_nbr)`

    # multiprocessing pool to parallel cluster pointcloud
    pool = multiproc.Pool(len(cluster_number))
    build_neighbors_func = partial(build_neighbors, args=deepcopy(args))
    k_nbrs = pool.map(build_neighbors_func, cluster_number)
    pool.close()
    pool.join()


    # get argmin k according E, different points may have different k
    k_entropys = []
    for k_nbr in k_nbrs:
        k_entropys.append(k_nbr['entropy'])
    argmink_ind = np.argmin(np.asarray(k_entropys), axis=0)




    points_feature = []
    for index in range(pointcloud.shape[0]):
        ### per point
        neighborhood = k_nbrs[argmink_ind[index]]['indices'][index]
        eigens_ = k_nbrs[argmink_ind[index]]['eigens_'][index]
        vectors_ = k_nbrs[argmink_ind[index]]['vectors_'][index]

        # calculate point feature
        feature = calculate_features(pointcloud, neighborhood, eigens_, vectors_)
        points_feature.append(feature)
    points_feature = np.asarray(points_feature)

    time3 = time.time()
    #print(points_feature)
    #print(points_feature.shape)
    #print("cluster time:{},feature_time:{},total_time:{}".format(time1-time0, time3-time1, time3-time0))
    return points_feature


def callback(ros_pc):

    timestamp = ros_pc.header.stamp.to_nsec()

    print('callback: msg : seq=%d, timestamp=%19d'%(ros_pc.header.seq, timestamp))
    start = time.time()
    gen = pc2.read_points(ros_pc, field_names=("x", "y", "z"), skip_nans=True)
    pointcloud = np.asarray(list(gen))

    #parse = argparse.ArgumentParser(sys.argv[0])
    #parse.add_argument('--k_start', type=int, default=20, 
    #                   help="KNN cluster k range start point")
    #parse.add_argument('--k_end', type=int, default=20,
    #                   help="KNN cluster k range end point")
    #parse.add_argument('--k_step', type=int, default=10,
    #                   help="KNN cluster k range step")
    #parse.add_argument('--bin_core_num', type=int, default=10, help="Parallel process file Pool core num")
    #args = parse.parse_args(sys.argv[1:])

    features=generate_feature(pointcloud, args)
    feature_points = np.hstack((pointcloud,features))
    end = time.time()
    #print(feature_points.shape)
    #print('calfeature time:{}'.format(end-start))

    feature_msg=feature_array_to_pointcloud2(feature_points)
    feature_msg.header.stamp = ros_pc.header.stamp
    feature_msg.header.frame_id = ros_pc.header.frame_id
    pubPointCloudMsgPlus.publish(feature_msg)

    ##np.save(file,arr)


if __name__=='__main__':

    rospy.init_node('keyFrameMsg_subscriber', anonymous=True)
    pubPointCloudMsgPlus=rospy.Publisher('/lpd_net/feature_pointclouds',PointCloud2,queue_size=3)
    velodyne_subscriber = rospy.Subscriber('/lpd_net/pionts_4096', PointCloud2, callback)  #key_frame_cloud
    rospy.spin()
