#include "myslam/frame.h"

namespace myslam
{
    Frame::Frame()
    : id_(-1), time_stamp_(-1), camera_(nullptr)
    {
    }

    Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
    : id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth)
    {
    }

    Frame::~Frame()
    {
    }

    Frame::Ptr Frame::createFrame()
    {
        static long factory_id = 0;
        return Frame::Ptr( new Frame(factory_id++) );
    }

    double Frame::findDepth ( const cv::KeyPoint& kp )
    {
        // cvRound()：返回跟参数最接近的整数值，即四舍五入；
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = depth_.ptr<ushort>(y)[x];
        if ( d!=0 )
        {
            return double(d)/camera_->depth_scale_;     // 如果深度信息存在，直接返回深度信息值
        }
        else 
        {
            // check the nearby points 
            int dx[4] = {-1,0,1,0};
            int dy[4] = {0,-1,0,1};
            for ( int i=0; i<4; i++ )
            {
                d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];  // 查找关键点上下左右的深度信息
                if ( d!=0 )
                {
                    return double(d)/camera_->depth_scale_;
                }
            }
        }
        return -1.0;
}


    Vector3d Frame::getCamCenter() const
    {
        return T_c_w_.inverse().translation();
    }

    // 判断世界坐标点是否在图像帧中
    bool Frame::isInFrame ( const Vector3d& pt_world )  // 输入世界坐标点位置
    {
        // typedef Eigen::Matrix<double, 3, 1> Eigen::Vector3d
        // Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
        // Scalar是表示元素的类型，RowsAtCompileTime为矩阵的行，ColsAtCompileTime为矩阵的列。
        Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );  // 世界坐标转相机坐标
        if ( p_cam(2,0)<0 ) 
            return false;
        Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
        return pixel(0,0)>0 && pixel(1,0)>0 
            && pixel(0,0)<color_.cols 
            && pixel(1,0)<color_.rows;
    }

}