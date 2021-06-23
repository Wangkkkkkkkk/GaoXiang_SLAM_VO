#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/frame.h"

namespace myslam
{
    class VisualOdometry
    {
    public:
        typedef boost::shared_ptr<VisualOdometry> Ptr;
        enum VOState{
            INITIALIZING=-1,
            OK=0,
            LOST
        };

        VOState                                                        state_;     // current VO status
        Map::Ptr                                                        map_;     // map with all frames and map points
        Frame::Ptr                                                        ref_;     // reference frame  参考帧
        Frame::Ptr                                                     curr_;     // current frame      当前帧

        cv::Ptr<cv::ORB>                                          orb_;    // orb detector and computer
        vector<cv::KeyPoint>            keypoints_curr_;   // keypoints in current frame
        Mat                                          descriptors_curr_;   // descriptor in current frame

        cv::FlannBasedMatcher      matcher_flann_;    // flann matcher
        vector<MapPoint::Ptr>           match_3dpts_;    // matched 3d points 
        vector<int>                      match_2dkp_index_;   // matched 2d pixels (index of kp_curr) 

        SE3                                          T_c_w_estimated_;   // the estimated pose of current frame
        int                                                      num_inliers_;   // number of inlier features in icp   icp内隐特征数
        int                                                           num_lost_;   // number of lost times

        // parameters
        int                                           num_of_features_;    // number of features
        double                                            scale_factor_;    // scale in image pyramid   图像金字塔比例尺
        int                                                level_pyramid_;     // number of pyramid levels  金字塔层级数
        float                                                 match_ratio_;    // ratio for selecting good matches
        int                                               max_num_lost_;    // max number of continuous lost times
        int                                                       min_inliers_;    // minimum inliers

        double                              key_frame_min_rot;      // minimal rotation of two key-frame
        double                         key_frame_min_trans;      // minimal translation of two key-frame
        double                   map_point_erase_ratio_;      // remove map point ratio

    public:
        VisualOdometry();
        ~VisualOdometry();

        // 提取特征点
        bool addFrame(Frame::Ptr frame);

    protected:
        // 提取特征点
        void extractKeyPoints();            
        // 计算描述子
        void computeDescriptors();     
        // 特征匹配
        void featureMatching();             
        // 位姿估计
        void poseEstimationPnP();      
        // 更新局部地图
        void optimizeMap();                    

        // 添加关键帧
        void addKeyFrame();                  
        // 添加地图点
        void addMapPoints();                 
        // 检查估计位姿是否准确
        bool checkEstimatedPose();   
        // 检查是否为关键帧
        bool checkKeyFrame();              

        double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
    };
}

#endif