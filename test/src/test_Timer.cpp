#include "test_Timer.hpp"
#include "ck_utilities/CKTimer.hpp"

#include "ros/ros.h"


#include <gtest/gtest.h>
#include <thread>

ros::NodeHandle *node;

TEST(CKUtilitiesTest, GTestTest)
{
    ASSERT_TRUE(true);
    ASSERT_FALSE(false);
}

TEST(CKUtilitiesTest, ElapsedTimerTest)
{
    ck::ElapsedTimer eTimer;
    eTimer.start();
    std::this_thread::sleep_for((std::chrono::milliseconds(1500)));
    double testVal = eTimer.hasElapsed();
    ASSERT_NEAR(testVal, 1.5, 0.1);
}

TEST(CKUtilitiesTest, TimeoutTimerTest)
{
    ck::TimeoutTimer timeoutTimer(1.7);
    ASSERT_FALSE(timeoutTimer.isTimedOut());
    std::this_thread::sleep_for((std::chrono::milliseconds(1300)));
    ASSERT_NEAR(timeoutTimer.getTimeLeft(), 0.4, 0.1);
    std::this_thread::sleep_for((std::chrono::milliseconds(500)));
    ASSERT_TRUE(timeoutTimer.isTimedOut());
    ASSERT_DOUBLE_EQ(timeoutTimer.getTimeLeft(), 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_ck_utilities_node");
    ros::NodeHandle nh;
    node = &nh;
    return RUN_ALL_TESTS();
}