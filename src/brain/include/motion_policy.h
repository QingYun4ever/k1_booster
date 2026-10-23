#pragma once

#include <chrono>
#include <limits>
#include <cmath>
#include <algorithm>

namespace motion_policy {

using MonotonicClock = std::chrono::steady_clock;
using ObservationStamp = std::chrono::nanoseconds;

enum class ProgressObservation {
    repeated,
    no_progress,
    progressed,
};

class VisualKickProgress {
public:
    void start(ObservationStamp observation, double range, MonotonicClock::time_point now) noexcept
    {
        _initialized = std::isfinite(range);
        if (!_initialized) return;
        _lastObservation = observation;
        _bestRange = range;
        _lastProgress = now;
    }

    ProgressObservation observe(ObservationStamp observation, double range,
        MonotonicClock::time_point now) noexcept
    {
        if (!_initialized || observation <= _lastObservation) return ProgressObservation::repeated;
        _lastObservation = observation;
        if (!std::isfinite(range) || _bestRange - range < kMinimumRangeDecrease) {
            return ProgressObservation::no_progress;
        }
        _bestRange = range;
        _lastProgress = now;
        return ProgressObservation::progressed;
    }

    bool stalled(MonotonicClock::time_point now, MonotonicClock::duration timeout) const noexcept
    {
        return _initialized && now - _lastProgress >= timeout;
    }

private:
    static constexpr double kMinimumRangeDecrease = 0.1;
    bool _initialized = false;
    ObservationStamp _lastObservation{0};
    double _bestRange = 0.0;
    MonotonicClock::time_point _lastProgress{};
};

inline bool shouldCancelVisualKick(bool motionAllowed, bool kickAllowed,
    bool freshLocalBall, bool kickoffGuardActive) noexcept
{
    return !motionAllowed || kickoffGuardActive || !freshLocalBall || !kickAllowed;
}

class ArrivalLock {
public:
    bool shouldHold(double targetX, double targetY, double targetTheta,
        double robotX, double robotY, double robotTheta,
        double distanceTolerance, double headingTolerance,
        double maxReleaseDistance = std::numeric_limits<double>::infinity()) noexcept
    {
        if (!std::isfinite(targetX) || !std::isfinite(targetY) || !std::isfinite(targetTheta)
            || !std::isfinite(robotX) || !std::isfinite(robotY) || !std::isfinite(robotTheta)
            || !std::isfinite(distanceTolerance) || !std::isfinite(headingTolerance)
            || distanceTolerance < 0.0 || headingTolerance < 0.0 || maxReleaseDistance < 0.0
            || std::isnan(maxReleaseDistance)) {
            _arrived = false;
            return false;
        }

        const double distance = std::hypot(targetX - robotX, targetY - robotY);
        const double headingError = std::fabs(wrapAngle(targetTheta - robotTheta));
        if (_arrived) {
            const double targetShift = std::hypot(targetX - _targetX, targetY - _targetY);
            if (targetShift > kTargetShiftTolerance
                || std::fabs(wrapAngle(targetTheta - _targetTheta)) > kTargetShiftTolerance) {
                _arrived = false;
            }
        }
        if (_arrived) {
            const double releaseDistance = std::min(maxReleaseDistance,
                distanceTolerance + std::max(0.15, distanceTolerance * 0.25));
            if (distance <= releaseDistance && headingError <= headingTolerance + kReleaseHeadingMargin) {
                return true;
            }
            _arrived = false;
        }
        if (distance <= distanceTolerance && headingError <= headingTolerance) {
            _arrived = true;
            _targetX = targetX;
            _targetY = targetY;
            _targetTheta = targetTheta;
            return true;
        }
        return false;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kTargetShiftTolerance = 0.1;
    static constexpr double kReleaseHeadingMargin = 0.1;

    static double wrapAngle(double angle) noexcept
    {
        return std::remainder(angle, 2.0 * kPi);
    }

    bool _arrived = false;
    double _targetX = 0.0;
    double _targetY = 0.0;
    double _targetTheta = 0.0;
};

} // namespace motion_policy
