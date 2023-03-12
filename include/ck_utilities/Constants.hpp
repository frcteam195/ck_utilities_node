#pragma once

// TODO: Move a lot of these constants to robot params.
namespace ck
{
    constexpr double kPathLookaheadTime = 0.25;
    constexpr double kPathMinLookaheadDistance = 12.0;
    constexpr double kAdaptivePathMinLookaheadDistance = 6.0;
    constexpr double kAdaptivePathMaxLookaheadDistance = 24.0;
    constexpr double kAdaptiveErrorLookaheadCoefficient = 0.01;
} // namespace ck