#include "myslam/common_include.h"

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"

namespace myslam
{
    VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_(new cv::flann::LshIndexParams(5, 10, 2))
    {

        num_of_features_        = Config::get<int> ( "number_of_features" );
        scale_factor_                   = Config::get<double> ( "scale_factor" );
        level_pyramid_              = Config::get<int> ( "level_pyramid" );
        match_ratio_                   = Config::get<float> ( "match_ratio" );
        max_num_lost_             = Config::get<float> ( "max_num_lost" );
        min_inliers_                     = Config::get<int> ( "min_inliers" );
        key_frame_min_rot      = Config::get<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }

    VisualOdometry::~VisualOdometry()
    {

    }

    bool VisualOdometry::addFrame ( Frame::Ptr frame )        // 新帧到来时位姿估计
    {
        switch ( state_ )
        {
        case INITIALIZING:   // 首帧状态
        {
            state_ = OK;  // 切换到正常状态
            curr_ = ref_ = frame;
            // extract features from first frame 
            extractKeyPoints();
            // compute the 3d position of features in ref frame 
            computeDescriptors();
            // the first frame is a key-frame
            addKeyFrame();  
            break;
        }
        case OK:
        {
            curr_ = frame;   // 当前帧=输入帧
            curr_->T_c_w_ = ref_->T_c_w_;    // 当前帧的位姿变换=参考帧的位姿变换
            boost::timer timer_ef;
            extractKeyPoints();
            cout<<"VO_extractfeature costs time:    "<<timer_ef.elapsed()<<endl;
            boost::timer timer_cd;
            computeDescriptors();
            cout<<"VO_computedescriptors costs time: "<<timer_cd.elapsed()<<endl;
            boost::timer timer_fm;
            featureMatching();
            cout<<"VO_featueematching costs time:   "<<timer_fm.elapsed()<<endl;
            poseEstimationPnP();
            if ( checkEstimatedPose() == true ) // a good estimation
            {
                curr_->T_c_w_ = T_c_w_estimated_;
                optimizeMap();
                num_lost_ = 0;
                if ( checkKeyFrame() == true ) // is a key-frame
                {
                    addKeyFrame();
                }
            }
            else // bad estimation due to various reasons
            {
                num_lost_++;
                if ( num_lost_ > max_num_lost_ )
                {
                    state_ = LOST;
                }
                return false;
            }
            break;
        }
        case LOST:          // 跟踪丢失
        {
            cout<<"vo has lost."<<endl;
            break;
        }
        }
        return true;
    }

    void VisualOdometry::extractKeyPoints()  // 提取特征点
    {
        orb_->detect ( curr_->color_, keypoints_curr_ );
    }

    void VisualOdometry::computeDescriptors()  // 计算描述子
    {
        orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    }

    void VisualOdometry::featureMatching()
    {
        // match desp_ref and desp_curr, use OpenCV's brute force match 
        vector<cv::DMatch> matches;       // 匹配点信息

        // select the candidates in map 
        Mat desp_map;
        vector<MapPoint::Ptr> candidate;

        // 将局部地图点提取出来，判断是否在当前帧内
        for ( auto allpoints = map_->map_points_.begin(); allpoints != map_->map_points_.end(); )
        {
            MapPoint::Ptr& p = allpoints->second;
            // check if p in curr frame image 
            if ( curr_->isInFrame(p->pos_) )
            {
                // add to candidate 
                p->visible_times_++;       // 如果是，该地图点可视次数加1
                candidate.push_back( p );   // 如果是，该地图点存入candidate
                desp_map.push_back( p->descriptor_ );  // 如果是，该地图点描述子信息存入desp_map
            }
            allpoints++;
        }

        // 使用OPENCV的快速最近邻匹配匹配特征点
        matcher_flann_.match ( desp_map, descriptors_curr_, matches );

        // select the best matches
        // min_element(first,end,cmp);其中cmp为可选择参数!
        // 第三个参数cmp可写可不写， 
        // max_element() 和 min_element() 默认是从小到大排列，
        // 然后 max_element() 输出最后一个值， min_element() 输出第一个值，
        // 但是如果自定义的 cmp 函数写的是从大到小排列，
        // 那么会导致 max_element() 和min_element() 的两个结果是对调的
        float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
        {
            return m1.distance < m2.distance;       // cmp函数定义从小到大排列
        } )->distance;        // 最小元素的距离

        match_3dpts_.clear();
        match_2dkp_index_.clear();

        // 获得合适的匹配
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
            {
                match_3dpts_.push_back( candidate[m.queryIdx] );   // 保存匹配点的3D信息
                match_2dkp_index_.push_back( m.trainIdx );                 // 保存匹配点的2D信息的index
            }
        }
        cout<<"good matches: "<<match_3dpts_.size()<<endl;
    }

void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    
    for ( int index:match_2dkp_index_ )     // 将匹配点的信息提取出来
    {
        pts2d.push_back ( keypoints_curr_[index].pt );  // 根据index找到当前帧特征点像素坐标
    }
    for ( MapPoint::Ptr pt:match_3dpts_ )
    {
        pts3d.push_back( pt->getPositionCV() );         // 根据地图点信息找到位置信息
    }

    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0,0,1
    );        // 相机内参
    
    Mat rvec, tvec, inliers;

/*
bool solvePnPRansac(InputArray _opoints, InputArray _ipoints,
	InputArray _cameraMatrix, InputArray _distCoeffs,
	OutputArray _rvec, OutputArray _tvec, bool useExtrinsicGuess,
	int iterationsCount, float reprojectionError, double confidence,
	OutputArray _inliers, int flags)

函数功能：用ransac的方式求解PnP问题

参数：
*   [in]    _opoints                      参考点在世界坐标系下的点集；float or double
*   [in]    _ipoints                        参考点在相机像平面的坐标；float or double
*   [in]    _cameraMatrix          相机内参
*   [in]    _distCoeffs                  相机畸变系数
*   [out]   _rvec                            旋转矩阵
*   [out]   _tvec                            平移向量
*   [in]    useExtrinsicGuess     若果求解PnP使用迭代算法，初始值可以使用猜测的初始值（true），也可以使用解析求解的结果作为初始值（false）。
*   [in]    iterationsCount         Ransac算法的迭代次数，这只是初始值，根据估计外点的概率，可以进一步缩小迭代次数；（此值函数内部是会不断改变的）,所以一开始可以赋一个大的值。
*   [in]    reprojectionErrr        Ransac筛选内点和外点的距离阈值，这个根据估计内点的概率和每个点的均方差（假设误差按照高斯分布）可以计算出此阈值。
*   [in]    confidence                   此值与计算采样（迭代）次数有关。此值代表从n个样本中取s个点，N次采样可以使s个点全为内点的概率。
*   [out]   _inliers                        返回内点的序列。为矩阵形式
*   [in]    flags                                最小子集的计算模型；
*                                                 SOLVEPNP_ITERATIVE(此方案，最小模型用的EPNP，内点选出之后用了一个迭代)；
*                                                 SOLVE_P3P(P3P只用在最小模型上，内点选出之后用了一个EPNP)      
*                                                 SOLVE_AP3P(AP3P只用在最小模型上，内点选出之后用了一个EPNP)
*                                                 SOLVE_EPnP(最小模型上&内点选出之后都采用了EPNP)
*    返回值：
*         成功返回true，失败返回false；
*/

    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers, cv::SOLVEPNP_EPNP ); // 计算位姿
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_w_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
    cout<<"T_c_r_estimated_= "<<endl<<T_c_w_estimated_<<endl;

   
    // using bundle adjustment to optimize the pose 
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 2> > Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense< Block::PoseMatrixType >();
    Block* solver_ptr = new Block( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_w_estimated_.rotation_matrix(), 
        T_c_w_estimated_.translation()
    ) );
    optimizer.addVertex ( pose );

    // edges
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int>(i,0);
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d( pts3d[index].x, pts3d[index].y, pts3d[index].z );
        edge->setMeasurement( Vector2d(pts2d[index].x, pts2d[index].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        optimizer.addEdge( edge );
        // set the inlier map points 
        match_3dpts_[index]->matched_times_++;  // 该点的匹配次数加1
    }
    
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    T_c_w_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );

}

bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_w_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_w_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    if(map_->keyframes_.empty())
    {
        // first key-frame, add all 3d points into map
        for(size_t i=0; i<keypoints_curr_.size(); i++){
            double d = curr_->findDepth(keypoints_curr_[i]);
            if( d < 0 )
            {
                continue;
            }
            Vector3d p_world = ref_->camera_->pixel2world(
                Vector2d (keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), curr_->T_c_w_, d
            );
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            );
            map_->insertMapPoint(map_point);
        }
    }
    cout<<"adding a key-frame"<<endl;
    map_->insertKeyFrame ( curr_ );
    ref_ = curr_;
}

void VisualOdometry::addMapPoints()
{
    // add the new map points into map
    vector<bool> matched(keypoints_curr_.size(), false); 
    for ( int index : match_2dkp_index_ )
        matched[index] = true;
    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
        if ( matched[i] == true )   
            continue;
        double d = ref_->findDepth ( keypoints_curr_[i] );
        if ( d<0 )  
            continue;
        Vector3d p_world = ref_->camera_->pixel2world (
            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
            curr_->T_c_w_, d
        );
        Vector3d n = p_world - ref_->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
        );
        map_->insertMapPoint( map_point );
    }
}

void VisualOdometry::optimizeMap()
{
    // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        // 3d点是否在图像帧内，不在图像帧内3d点删除
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        
        // 3d点的匹配次数/可视次数，如果过小，认为是无意义点
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
       
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }
    
    if ( match_2dkp_index_.size()<100 )
        addMapPoints();
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}

double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos( n.transpose()*point->norm_ );
}

}