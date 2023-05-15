#!/usr/bin/env python3
# tensorflow
import tensorflow as tensorf
import numpy as np
import importlib
import time
# ros
import rospy
from sensor_msgs.msg import PointCloud2
from gp_lio.msg import FeatureVector
from sensor_msgs.msg import Joy
from sensor_msgs import point_cloud2
from std_msgs.msg import String

class GetFeatureVectorByLpdnet():
    """This class is to get feature vector encode
    from pointcloud,which is used for closure dection"""
    def __init__(self):
        self.begin = time.time()
        rospy.init_node("lpdnet_node")
        self.handcraft_feature = np.asarray([])
        self.encode_vec = []

        # topics
        self.lidar_topic = "/lpd_net/feature_pointclouds"
        self.feature_vector_topic = "/lpd_net/feature_vector"

        # load pretrained model
        rospy.loginfo("start initialization ...")
        self.gpu = 0
        self.NUM_POINTS = 4096
        self.EVAL_BATCH_SIZE = 1
        self.positives_per_query = 0
        self.negatives_per_query = 0
        self.batch_num_queries = 1
        self.dimension = 256
        self.decay_step =200000
        self.decay_rate = 0.7
        self.model = "lpd_FNSF"
        self.model_file = "/home/xc/catkin_ws/src/gp_lio/scripts/lpd_net/log/lpd_FNSF-18-11-16-12-03/model.ckpt"
        self.MODEL = importlib.import_module(self.model)
        self.graph = tensorf.Graph()
        self.query = []
        self.positives = []
        self.negatives = []
        self.eval_queries = []
        self.is_training_pl = []
        self.saver = []
        self.sess = []

        self.q_vec = []
        self.pos_vecs = []
        self.neg_vecs = []

        self.BN_INIT_DECAY = 0.5
        self.BN_DECAY_DECAY_RATE = 0.5
        self.BN_DECAY_DECAY_STEP = float(self.decay_step)
        self.BN_DECAY_CLIP = 0.99

        with self.graph.as_default():
            with tensorf.device('/gpu:' + str(self.gpu)):
                self.query = self.MODEL.placeholder_inputs(self.batch_num_queries, 1, self.NUM_POINTS)
                self.positives = self.MODEL.placeholder_inputs(self.batch_num_queries, self.positives_per_query, self.NUM_POINTS)
                self.negatives = self.MODEL.placeholder_inputs(self.batch_num_queries, self.negatives_per_query, self.NUM_POINTS)
                self.eval_queries = self.MODEL.placeholder_inputs(self.EVAL_BATCH_SIZE, 1, self.NUM_POINTS)
                self.is_training_pl = tensorf.placeholder(tensorf.bool, shape=())
                batch = tensorf.Variable(0)
                bn_decay = self.get_bn_decay(batch)

                with tensorf.variable_scope("query_triplets") as scope:
                    vecs = tensorf.concat([self.query, self.positives, self.negatives], 1)
                    out_vecs = self.MODEL.forward(vecs, self.is_training_pl, bn_decay=bn_decay)
                    self.q_vec, self.pos_vecs, self.neg_vecs = tensorf.split(out_vecs, [1, self.positives_per_query, self.negatives_per_query], 1)
                # Create a session
                gpu_options = tensorf.GPUOptions(per_process_gpu_memory_fraction=0.95)
                config = tensorf.ConfigProto(gpu_options=gpu_options)
                config.gpu_options.allow_growth = True
                config.allow_soft_placement = True
                config.log_device_placement = False
                self.sess = tensorf.Session(config=config)
                self.saver = tensorf.train.Saver()
                self.saver.restore(self.sess, self.model_file)
        rospy.loginfo("Model has loaded ...")
        # ros
        self._sub = rospy.Subscriber(self.lidar_topic,PointCloud2,self.processPointclouds,queue_size=10)
        self._pub = rospy.Publisher(self.feature_vector_topic,FeatureVector,queue_size=1)
        rospy.loginfo("Initialization has finished ...")


    def get_bn_decay(self, batch):
        bn_momentum = tensorf.train.exponential_decay(
            self.BN_INIT_DECAY,
            batch * self.batch_num_queries,
            self.BN_DECAY_DECAY_STEP,
            self.BN_DECAY_DECAY_RATE,
            staircase=True)
        bn_decay = tensorf.minimum(self.BN_DECAY_CLIP, 1 - bn_momentum)
        return bn_decay
    
    
    def processPointclouds(self,featureclouds2):
        "callBack"
        pointcloud_iter = point_cloud2.read_points(featureclouds2)
        print("Read new kf feature cloud ")
        points = []
        for xyz in pointcloud_iter:
            points.append(xyz)
        self.handcraft_feature = np.asarray(points)
        with self.graph.as_default():
            ops = {'query': self.query,
                   'positives': self.positives,
                   'negatives': self.negatives,
                   'is_training_pl': self.is_training_pl,
                   'eval_queries': self.eval_queries,
                   'q_vec': self.q_vec,
                   'pos_vecs': self.pos_vecs,
                   'neg_vecs': self.neg_vecs}
            encode_vec = self.get_latent_vectors(self.sess, ops)
            
            self.encode_vec = FeatureVector()
            self.encode_vec.header.stamp = featureclouds2.header.stamp
            for i in range(self.dimension):
                # print(encode_vec[0][i])
                self.encode_vec.data.append(encode_vec[0][i])
            # self.encode_vec.data = encode_vec.tolist()
            self._pub.publish(self.encode_vec)


    def get_latent_vectors(self,sess, ops):
        rospy.loginfo("processing cloud using network ...")
        is_training = False
        queries = self.handcraft_feature
        # preprocessing data
        # Normalization
        queries = ((queries - queries.min(axis=0)) / (queries.max(axis=0) - queries.min(axis=0)))
        queries[np.isnan(queries)] = 0.0
        queries[np.isinf(queries)] = 1.0
        q1 = queries
        q1 = np.reshape(q1,(self.batch_num_queries,1,self.NUM_POINTS,13))
        q2 = queries[self.batch_num_queries:self.batch_num_queries * (self.positives_per_query + 1)]
        q2 = np.reshape(q2, (self.batch_num_queries, self.positives_per_query, self.NUM_POINTS, 13))
        q3 = queries[self.batch_num_queries * (self.positives_per_query + 1):self.batch_num_queries * (
                    self.negatives_per_query + self.positives_per_query + 1)]
        q3 = np.reshape(q3, (self.batch_num_queries, self.negatives_per_query, self.NUM_POINTS, 13))
        feed_dict = {ops['query']: q1, ops['positives']: q2, ops['negatives']: q3,
                     ops['is_training_pl']: is_training}
        o1, o2, o3 = sess.run([ops['q_vec'], ops['pos_vecs'], ops['neg_vecs']], feed_dict=feed_dict)
        o1 = np.reshape(o1, (-1, o1.shape[-1]))
        o2 = np.reshape(o2, (-1, o2.shape[-1]))
        o3 = np.reshape(o3, (-1, o3.shape[-1]))
        #print(o1.shape," ",o2.shape," ",o3.shape)

        q_output = np.vstack((o1, o2, o3))
        #print("\n",q_output.shape)
        return q_output


if __name__ =="__main__":
    try:
        GetFeatureVectorByLpdnet()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("GetFeatureVectorByLpdnet has started.")







