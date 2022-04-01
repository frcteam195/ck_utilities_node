#include "ck_utilities/RateControlledPublisher.hpp"
#include <ck_utilities_node/TestMessage.h>

#include "ros/ros.h"

#include <gtest/gtest.h>
#include <thread>

extern ros::NodeHandle *node;

// TEST(CKUtilitiesTest, ROSRatePub)
// {
//     ck_utilities_node::TestMessage msg;
//     msg.test_data = "test";
//     ck::ros::RateControlledPublisher<ck_utilities_node::TestMessage> testPub("/TestTopic", 1);
//     testPub.publish_at_rate(msg, 10);
//     //While this is sleeping, measure the rate using rostopic hz /TestTopic
//     std::this_thread::sleep_for(std::chrono::milliseconds(20000));
//     ASSERT_EQ(true, true);
// }