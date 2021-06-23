#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// use Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using Eigen::Vector2d;
using Eigen::Vector3d;

// use sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;

// use opencv
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp> 
using cv::Mat;

// use g2o
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <fstream>

#include "boost/shared_ptr.hpp"
#include "boost/unordered_map.hpp"


#include "boost/timer.hpp"

using namespace std;

#endif   // COMMON_INCLUDE_H