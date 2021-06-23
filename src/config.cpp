#include "myslam/config.h"

namespace myslam 
{
    
void Config::setParameterFile( const std::string& filename )
{
    if ( config_ == nullptr )
        config_ = boost::shared_ptr<Config>(new Config);
    
    // 使用Opencv提供的FileStorage类。可以提取一个YAML文件，且可以访问其中任意一个字段
    // 由于参数实质值可能为整数、浮点数或字符串，所以通过一个模板函数get来获得任意类型的参数值
    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if ( config_->file_.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return;
    }
}

Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
}

boost::shared_ptr<Config> Config::config_ = nullptr;

}