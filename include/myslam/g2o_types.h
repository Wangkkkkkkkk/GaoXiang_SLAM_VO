#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{
    class EdgeProjectXYZRGBD: public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void computeError();
        virtual void linearizeOplus();
        virtual bool read( std::istream& in ){}
        virtual bool write( std::ostream& out) const {}
    };

    // only to optimize the pose, no point
    class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap >
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Error: measure = R*point+t
        virtual void computeError();
        virtual void linearizeOplus();
    
        virtual bool read( std::istream& in ){}
        virtual bool write( std::ostream& out) const {}
    
        Vector3d point_;
    };

    class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
        virtual void computeError();
        virtual void linearizeOplus();
    
        virtual bool read( std::istream& in ){}
        virtual bool write(std::ostream& os) const {};
    
        Vector3d point_;
        Camera* camera_;
    };
}

#endif