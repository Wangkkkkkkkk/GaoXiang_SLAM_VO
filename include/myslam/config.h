/*
Config类负责参数文件的读取，并在程序的任意地方都可随时提供参数的值
它只有一个全局对象，当我们设置参数文件时，创建该对象并读取参数文件
随后就可以在任意地方访问参数值，最后在程序结束时自动销毁
*/

#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    static boost::shared_ptr<Config> config_; 
    cv::FileStorage file_;
    
    Config () {} // private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing 
    
    // set a new config file 
    static void setParameterFile( const std::string& filename ); 
    
    // access the parameter values  访问参数值
    template< typename T >
    static T get( const std::string& key )
    {
        // 使用Opencv提供的FileStorage类。可以提取一个YAML文件，且可以访问其中任意一个字段
        // 由于参数实质值可能为整数、浮点数或字符串，所以通过一个模板函数get来获得任意类型的参数值
        return T( Config::config_->file_[key] );
    }
};
}

#endif // CONFIG_H