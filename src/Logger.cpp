#include "ck_utilities/Logger.hpp"
#include "ros/ros.h"

#include <sstream>

enum class ros_log_level
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
};

class logger : public std::stringbuf
{
public:
    logger(ros_log_level level)
    {
        this->log_level = level;
    }

    virtual int sync()
    {
        switch(this->log_level)
        {
            case ros_log_level::DEBUG:
            {
                ROS_DEBUG("%s", this->str().c_str());
                break;
            }
            case ros_log_level::INFO:
            {
                ROS_INFO("%s", this->str().c_str());
                break;
            }
            case ros_log_level::WARN:
            {
                ROS_WARN("%s", this->str().c_str());
                break;
            }
            case ros_log_level::ERROR:
            {
                ROS_ERROR("%s", this->str().c_str());
                break;
            }
            case ros_log_level::FATAL:
            {
                ROS_FATAL("%s", this->str().c_str());
                break;
            }
        }
        this->str("");
        return 0;
    }

private:
    ros_log_level log_level;
};

logger debug_buf(ros_log_level::DEBUG);
logger info_buf(ros_log_level::INFO);
logger warn_buf(ros_log_level::WARN);
logger error_buf(ros_log_level::ERROR);
logger fatal_buf(ros_log_level::FATAL);

std::ostream ck::log_debug(&debug_buf);
std::ostream ck::log_info(&info_buf);
std::ostream ck::log_warn(&warn_buf);
std::ostream ck::log_error(&error_buf);
std::ostream ck::log_fatal(&fatal_buf);