#include "ck_utilities/MovingAverage.hpp"

#include <gtest/gtest.h>
#include <thread>

TEST(CKUtilitiesTest, MA1)
{
    ck::MovingAverage m(10);
}