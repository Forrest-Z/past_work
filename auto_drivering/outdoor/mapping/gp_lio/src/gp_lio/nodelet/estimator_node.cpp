#include "gp_lio/nodelet/EstimatorNodelet.h"

using namespace gp_lio;

int main(int argc, char **argv){

    ros::init(argc, argv, "estimator_node");
    EstimatorNodelet estimator;

    return 0;
}