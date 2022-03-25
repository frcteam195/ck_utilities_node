#include "ck_utilities/MovingAverage.hpp"

#include "ros/ros.h"

#include <gtest/gtest.h>
#include <thread>

extern ros::NodeHandle *node;

TEST(CKUtilitiesTest, MA1)
{
    ck::MovingAverage m(10);
    m.addSample(1);
    m.addSample(2);
    m.addSample(3);
    m.addSample(4);
    ASSERT_EQ(m.getAverage(), 2.5);
    m.addSample(5);
    m.addSample(6);
    m.addSample(7);
    m.addSample(8);
    m.addSample(9);
    m.addSample(10);
    ASSERT_EQ(m.getAverage(), 5.5);
    m.addSample(11);
    ASSERT_EQ(m.getAverage(), 6.5);
    m.addSample(55);
    ASSERT_EQ(m.getAverage(), 11.8);
    m.addSample(-22.2);
    ASSERT_EQ(m.getAverage(), 9.28);
}