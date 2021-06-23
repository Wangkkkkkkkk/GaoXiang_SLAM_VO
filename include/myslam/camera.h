#ifndef CAMERA_H
#define CAMERA_H
// 防止重复被引用

#include "myslam/common_include.h"

namespace myslam
{
    // Pinhole RGB-D camera model
    class Camera
    {
    public:
        // shared_ptr 是C++11提供的一种智能指针类，它足够智能，
        // 可以在任何地方都不使用时自动删除相关指针，从而帮助彻底消除内存泄漏和悬空指针的问题。
        typedef boost::shared_ptr<Camera> Ptr;
        float fx_, fy_, cx_, cy_, depth_scale_;  // Camera intrinsics 相机内参

        Camera();
        Camera( float fx, float fy, float cx, float cy, float depth_scale=0):
            fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale)
            {}
        
        // cordinate transform: world, camera, pixel
        Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
        Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w);
        Vector2d camera2pixel( const Vector3d& p_c);
        Vector3d pixel2camera( const Vector2d& p_p, double depth=1);
        Vector3d pixel2world( const Vector2d& p_p, const SE3& T_c_w, double depth=1);
        Vector2d world2pixel( const Vector3d& p_w, const SE3& T_c_w);
    };
}
#endif  // CAMERA_H