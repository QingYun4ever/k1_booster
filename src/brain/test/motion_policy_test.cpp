#include <cassert>
#include <chrono>
#include <cmath>

#include "motion_policy.h"

namespace {

using motion_policy::ArrivalLock;
using motion_policy::MonotonicClock;
using motion_policy::ObservationStamp;
using motion_policy::ProgressObservation;
using motion_policy::VisualKickProgress;

void testProgressUsesDistinctObservationsAndRangeDecrease()
{
    const auto start = MonotonicClock::time_point{};
    VisualKickProgress progress;
    progress.start(ObservationStamp{100}, 2.0, start);

    assert(progress.observe(ObservationStamp{100}, 1.0, start + std::chrono::seconds(1))
        == ProgressObservation::repeated);
    assert(progress.observe(ObservationStamp{200}, 2.2, start + std::chrono::seconds(1))
        == ProgressObservation::no_progress);
    assert(progress.observe(ObservationStamp{300}, 1.95, start + std::chrono::milliseconds(1500))
        == ProgressObservation::no_progress);
    assert(progress.stalled(start + std::chrono::seconds(2), std::chrono::seconds(2)));

    assert(progress.observe(ObservationStamp{400}, 1.8, start + std::chrono::milliseconds(2100))
        == ProgressObservation::progressed);
    assert(!progress.stalled(start + std::chrono::milliseconds(4099), std::chrono::seconds(2)));
    assert(progress.stalled(start + std::chrono::milliseconds(4100), std::chrono::seconds(2)));
}

void testOwnerLossAndPauseRequestCancellation()
{
    assert(motion_policy::shouldCancelVisualKick(false, true, true, false));
    assert(motion_policy::shouldCancelVisualKick(true, false, true, false));
    assert(motion_policy::shouldCancelVisualKick(true, true, true, true));
    assert(motion_policy::shouldCancelVisualKick(true, false, false, false));
    assert(!motion_policy::shouldCancelVisualKick(true, true, true, false));
}

void testArrivalLockHysteresisAndTargetChange()
{
    ArrivalLock lock;
    assert(lock.shouldHold(0.0, 0.0, 0.0, 0.49, 0.0, 0.0, 0.5, 0.1));
    assert(lock.shouldHold(0.0, 0.0, 0.0, 0.64, 0.0, 0.0, 0.5, 0.1));
    assert(!lock.shouldHold(0.0, 0.0, 0.0, 0.66, 0.0, 0.0, 0.5, 0.1));
    ArrivalLock targetChange;
    assert(targetChange.shouldHold(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.1));
    assert(!targetChange.shouldHold(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.1));

    ArrivalLock goalLineLock;
    assert(goalLineLock.shouldHold(0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.1, 0.05));
    assert(!goalLineLock.shouldHold(0.05, 0.0, 0.0, -0.01, 0.0, 0.0, 0.05, 0.1, 0.05));
    ArrivalLock invalidTolerance;
    assert(!invalidTolerance.shouldHold(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01, 0.1));
}

} // namespace

int main()
{
    testProgressUsesDistinctObservationsAndRangeDecrease();
    testOwnerLossAndPauseRequestCancellation();
    testArrivalLockHysteresisAndTargetChange();
    return 0;
}
