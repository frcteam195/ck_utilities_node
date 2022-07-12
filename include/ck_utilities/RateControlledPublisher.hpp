#pragma once
#if __has_include("ros/ros.h")
#include "ros/ros.h"
#include <mutex>
#include <atomic>
#include <thread>
#include <string>

extern ros::NodeHandle* node;

namespace ck
{
    namespace ros
    {
        template <typename T>
        class RateControlledPublisher
        {
        private:
            std::recursive_mutex m_pub_lock;
            T m_msg;
            std::atomic_bool run_thread {false};
            std::atomic_uint32_t m_thread_rate_hz {10};
            std::thread m_thread;
            ::ros::Publisher m_ros_pub;

            void run_publish_thread()
            {
                ::ros::Rate rate(m_thread_rate_hz);
                uint32_t previous_publish_rate = m_thread_rate_hz;
                while (::ros::ok() && run_thread)
                {
                    if (previous_publish_rate != m_thread_rate_hz)
                    {
                        rate = ::ros::Rate(m_thread_rate_hz);
                        previous_publish_rate = m_thread_rate_hz;
                    }

                    {
                        std::lock_guard<std::recursive_mutex> lock(m_pub_lock);
                        m_ros_pub.publish(m_msg);
                    }

                    rate.sleep();
                }
            }

        public:
            RateControlledPublisher(std::string topic_name, int queue_size = 1)
            {
                m_ros_pub = node->advertise<T>(topic_name, queue_size);
            }
            ~RateControlledPublisher()
            {
                run_thread = false;
                if (m_thread.joinable())
                {
                    m_thread.join();
                }
            }

            void publish_at_rate(T& msg, uint32_t publish_rate_hz)
            {
                {
                    std::lock_guard<std::recursive_mutex> lock(m_pub_lock);
                    m_msg = msg;
                }
                
                m_thread_rate_hz = publish_rate_hz;
                if (!run_thread)
                {
                    run_thread = true;
                    m_thread = std::thread(&RateControlledPublisher::run_publish_thread, this);
                }
            }
        };
 
    }
}
#endif