#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam 
{
    class MapPoint;
    class Frame
    {
    public:
        typedef boost::shared_ptr<Frame> Ptr;
        unsigned long                            id_;                    // 帧id
        double                      time_stamp_;                   // 时间戳
        SE3                                        T_c_w_;                   // 位姿
        Camera::Ptr                     camera_;                  // 相机
        Mat                          color_, depth_;

    public:
        Frame();
        Frame( long id, double time_stamp=0.0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat());
        ~Frame();

        // factory function
        static Frame::Ptr createFrame();

        // find the depth in depth map
        double findDepth( const cv::KeyPoint& kp);

        // Get Camera Center
        Vector3d getCamCenter() const;

        // check if a point is in this frame
        bool isInFrame( const Vector3d& pt_world);
    };
}

#endif  // FRAME_H