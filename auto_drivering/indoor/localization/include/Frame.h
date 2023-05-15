//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_FRAME_H
#define SRC_FRAME_H

#include <pcl/common/transforms.h>

//#include "LidarFeature.h"
#include "Utility.h"

namespace gp_lio
{

class Frame
{

public:
    Frame();

    Frame(State &state, CloudFeature &cloud_feature);

    ~Frame();


    PointCloudT GetTransformedGroundCloud(ExternalParameters &extrinsic_parameters);

    PointCloudT GetTransformedPlaneCloud(ExternalParameters &extrinsic_parameters);

    PointCloudT GetTransformedSurfCloud(ExternalParameters &extrinsic_parameters);

    PointCloudT GetTransformedEdgeCloud(ExternalParameters &extrinsic_parameters);

    std::vector<PlaneFeature> GetTransformedPlane(ExternalParameters &extrinsic_parameters);

    void SetAssociationPairsGroup(FeatureCorPairsGroup &edgePairs,
                                  FeatureCorPairsGroup &surfPairs);
    void SetAssociationGroundPairsGroup(FeatureCorPairsGroup &groundPairs);
    
    void SetAssociationPairsGroupForward(FeatureCorPairsGroup &edgePairs,
                                         FeatureCorPairsGroup &surfPairs);

    void SetAssociationPlaneGroup(PlaneCorPairsGroup planePairGroup);

    void SetCloud(CloudFeature &cloud_feature);

    void swap(Frame frame);

    void clear();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State state_;
    CloudFeature cloud_feature_;
    
    FeatureCorPairsGroup edgePairsGroup_;
    FeatureCorPairsGroup surfPairsGroup_;
    FeatureCorPairsGroup groundPairsGroup_;

    PlaneCorPairsGroup planeCorPairsGroup_;

    FeatureCorPairsGroup edgePairsGroupForward_;
    FeatureCorPairsGroup surfPairsGroupForward_;

};
} // namespace gp_lio

#endif //SRC_FRAME_H
