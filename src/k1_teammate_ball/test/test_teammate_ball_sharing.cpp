// 独立原生回归测试, 不依赖 ROS。
//
// 手动编译运行:
//   g++ -std=c++17 -Isrc/k1_teammate_ball/include src/k1_teammate_ball/src/teammate_ball_sharing.cpp src/k1_teammate_ball/test/test_teammate_ball_sharing.cpp -o /tmp/test_teammate_ball && /tmp/test_teammate_ball

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <limits>

#include "k1_teammate_ball/teammate_ball_sharing.hpp"

static std::size_t g_allocation_count = 0;

void *operator new(std::size_t size) {
    ++g_allocation_count;
    if (void *memory = std::malloc(size)) return memory;
    throw std::bad_alloc();
}

void *operator new[](std::size_t size) {
    ++g_allocation_count;
    if (void *memory = std::malloc(size)) return memory;
    throw std::bad_alloc();
}

void operator delete(void *memory) noexcept { std::free(memory); }
void operator delete[](void *memory) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void *memory, std::size_t) noexcept { std::free(memory); }

using k1_teammate_ball::BallReport;
using k1_teammate_ball::Config;
using k1_teammate_ball::FusedBall;
using k1_teammate_ball::TeammateBallSharing;

static BallReport makeReport(int player_id, double stamp_sec, double x = 5.0, double y = 0.0) {
    BallReport report;
    report.player_id = player_id;
    report.x = x;
    report.y = y;
    report.confidence = 80.0;
    report.range = 1.0;
    report.detected = true;
    report.location_known = true;
    report.robot_x = 0.0;
    report.robot_y = 0.0;
    report.stamp_sec = stamp_sec;
    return report;
}

static Config enabledConfig() {
    Config config;
    config.enable = true;
    config.min_confidence_to_share = 50.0;
    config.min_confidence_to_trust = 50.0;
    config.self_dist_threshold = 2.0;
    config.teammate_timeout_sec = 1.0;
    config.trust_timeout_sec = 1.0;
    return config;
}

static void testShareGate() {
    TeammateBallSharing sharing;
    Config config = enabledConfig();
    sharing.setConfig(config);
    assert(sharing.shouldShare(50.0, true));
    assert(!sharing.shouldShare(49.9, true));
    assert(!sharing.shouldShare(50.0, false));
    assert(!sharing.shouldShare(std::nan(""), true));
    assert(!sharing.shouldShare(100.1, true));

    config.enable = false;
    sharing.setConfig(config);
    assert(!sharing.shouldShare(90.0, true));
}

static void testFiniteInputAndLocalizationGates() {
    TeammateBallSharing sharing;
    sharing.setConfig(enabledConfig());

    BallReport invalid = makeReport(1, 40.0);
    invalid.x = std::numeric_limits<double>::infinity();
    sharing.ingest(invalid, 40.0);
    invalid = makeReport(2, 40.0);
    invalid.y = std::nan("");
    sharing.ingest(invalid, 40.0);
    invalid = makeReport(3, 40.0);
    invalid.confidence = 100.1;
    sharing.ingest(invalid, 40.0);
    invalid = makeReport(4, 40.0);
    invalid.confidence = -0.1;
    sharing.ingest(invalid, 40.0);
    invalid = makeReport(5, 40.0);
    invalid.range = -1.0;
    sharing.ingest(invalid, 40.0);
    invalid = makeReport(6, 40.0);
    invalid.range = std::numeric_limits<double>::infinity();
    sharing.ingest(invalid, 40.0);
    invalid = makeReport(7, 40.0);
    invalid.robot_x = std::nan("");
    sharing.ingest(invalid, 40.0);
    invalid = makeReport(8, std::numeric_limits<double>::infinity());
    sharing.ingest(invalid, 40.0);
    invalid = makeReport(0, 40.0);
    sharing.ingest(invalid, 40.0);

    sharing.ingest(makeReport(10, 40.0), std::nan(""));
    sharing.ingest(makeReport(11, 40.0), std::numeric_limits<double>::infinity());
    assert(!sharing.fuse(0.0, 0.0, true, 40.0).reliable);

    sharing.ingest(makeReport(9, 40.0), 40.0);
    assert(!sharing.fuse(0.0, 0.0, false, 40.0).reliable);
    assert(!sharing.fuse(std::nan(""), 0.0, true, 40.0).reliable);
    assert(!sharing.fuse(0.0, std::numeric_limits<double>::infinity(), true, 40.0).reliable);
    assert(!sharing.fuse(0.0, 0.0, true, std::nan("")).reliable);
    FusedBall result = sharing.fuse(0.0, 0.0, true, 40.0);
    assert(result.reliable && result.is_fresh);
    assert(result.source_player_id == 9);
    assert(result.source_stamp_sec == 40.0);
    assert(!sharing.fuse(5.0, 0.0, true, 40.1).reliable);
}

static void testDuplicateOldAndFreshOnceWithoutAllocations() {
    TeammateBallSharing sharing;
    sharing.setConfig(enabledConfig());

    BallReport accepted = makeReport(3, 10.0, 5.0);
    accepted.confidence = 80.0;
    sharing.ingest(accepted, 10.0);

    BallReport duplicate = makeReport(3, 10.0, 9.0);
    duplicate.confidence = 100.0;
    sharing.ingest(duplicate, 10.0);
    BallReport old = makeReport(3, 9.0, 8.0);
    old.confidence = 99.0;
    sharing.ingest(old, 10.0);

    const std::size_t allocations_before_fuse = g_allocation_count;
    FusedBall result = sharing.fuse(0.0, 0.0, true, 10.0);
    assert(g_allocation_count == allocations_before_fuse);
    assert(result.reliable && result.is_fresh);
    assert(result.x == 5.0 && result.confidence == 80.0);
    assert(result.source_stamp_sec == 10.0);

    result = sharing.fuse(0.0, 0.0, true, 10.1);
    assert(result.reliable && !result.is_fresh);
    assert(result.x == 5.0 && result.source_stamp_sec == 10.0);

    BallReport newer = makeReport(3, 10.2, 6.0);
    newer.confidence = 85.0;
    sharing.ingest(newer, 10.2);
    result = sharing.fuse(0.0, 0.0, true, 10.2);
    assert(result.reliable && result.is_fresh);
    assert(result.x == 6.0 && result.source_stamp_sec == 10.2);
}

static void testDeterministicTieAndFutureRejection() {
    TeammateBallSharing sharing;
    sharing.setConfig(enabledConfig());

    BallReport later_id = makeReport(8, 30.0, 6.0);
    later_id.confidence = 90.0;
    later_id.range = 0.8;
    BallReport lower_id = makeReport(2, 30.0, 7.0);
    lower_id.confidence = 90.0;
    lower_id.range = 0.8;
    sharing.ingest(later_id, 30.0);
    sharing.ingest(lower_id, 30.0);
    FusedBall result = sharing.fuse(0.0, 0.0, true, 30.0);
    assert(result.reliable && result.source_player_id == 2);

    BallReport future = makeReport(2, 30.1, 20.0);
    future.confidence = 100.0;
    sharing.ingest(future, 30.0);
    result = sharing.fuse(0.0, 0.0, true, 30.0);
    assert(result.reliable && !result.is_fresh);
    assert(result.source_player_id == 2 && result.source_stamp_sec == 30.0);

    TeammateBallSharing initial_future;
    initial_future.setConfig(enabledConfig());
    initial_future.ingest(makeReport(1, 30.1), 30.0);
    assert(!initial_future.fuse(0.0, 0.0, true, 30.0).reliable);
    initial_future.ingest(makeReport(1, 30.0, 5.0), 30.0);
    result = initial_future.fuse(0.0, 0.0, true, 30.0);
    assert(result.reliable && result.is_fresh && result.source_stamp_sec == 30.0);
    TeammateBallSharing between_ticks;
    between_ticks.setConfig(enabledConfig());
    between_ticks.ingest(makeReport(4, 100.0), 100.0);
    result = between_ticks.fuse(0.0, 0.0, true, 100.0);
    assert(result.reliable && result.is_fresh && result.source_stamp_sec == 100.0);
    between_ticks.ingest(makeReport(4, 100.1, 6.0), 100.1);
    result = between_ticks.fuse(0.0, 0.0, true, 100.1);
    assert(result.reliable && result.is_fresh && result.source_stamp_sec == 100.1);
    assert(result.x == 6.0);
}

static void testHoldAnchoredToObservationTime() {
    TeammateBallSharing sharing;
    Config config = enabledConfig();
    config.teammate_timeout_sec = 0.1;
    config.trust_timeout_sec = 0.5;
    sharing.setConfig(config);
    sharing.ingest(makeReport(1, 20.0), 20.0);

    FusedBall result = sharing.fuse(0.0, 0.0, true, 20.0);
    assert(result.reliable && result.is_fresh && result.source_stamp_sec == 20.0);
    result = sharing.fuse(0.0, 0.0, true, 20.05);
    assert(result.reliable && !result.is_fresh && result.source_stamp_sec == 20.0);

    result = sharing.fuse(0.0, 0.0, true, 20.2);
    assert(result.reliable && !result.is_fresh && result.source_stamp_sec == 20.0);
    result = sharing.fuse(0.0, 0.0, true, 20.49);
    assert(result.reliable && !result.is_fresh && result.source_stamp_sec == 20.0);
    result = sharing.fuse(0.0, 0.0, true, 20.51);
    assert(!result.reliable);
    TeammateBallSharing short_hold;
    config = enabledConfig();
    config.trust_timeout_sec = 0.1;
    config.teammate_timeout_sec = 1.0;
    short_hold.setConfig(config);
    short_hold.ingest(makeReport(2, 50.0), 50.0);
    assert(short_hold.fuse(0.0, 0.0, true, 50.0).reliable);
    assert(!short_hold.fuse(0.0, 0.0, true, 50.11).reliable);
}

static void testBackwardTimeReset() {
    TeammateBallSharing sharing;
    sharing.setConfig(enabledConfig());
    sharing.ingest(makeReport(1, 100.0), 100.0);
    FusedBall result = sharing.fuse(0.0, 0.0, true, 100.0);
    assert(result.reliable && result.is_fresh);

    result = sharing.fuse(0.0, 0.0, true, 99.0);
    assert(!result.reliable);
    sharing.ingest(makeReport(1, 99.0, 6.0), 99.0);
    result = sharing.fuse(0.0, 0.0, true, 99.0);
    assert(result.reliable && result.is_fresh);
    assert(result.x == 6.0 && result.source_stamp_sec == 99.0);
    TeammateBallSharing ingest_clock_reset;
    ingest_clock_reset.setConfig(enabledConfig());
    ingest_clock_reset.ingest(makeReport(2, 200.0), 200.0);
    assert(ingest_clock_reset.fuse(0.0, 0.0, true, 200.0).reliable);
    ingest_clock_reset.ingest(makeReport(2, 199.0, 7.0), 199.0);
    result = ingest_clock_reset.fuse(0.0, 0.0, true, 199.0);
    assert(result.reliable && result.is_fresh);
    assert(result.x == 7.0 && result.source_stamp_sec == 199.0);
}

int main() {
    testShareGate();
    testFiniteInputAndLocalizationGates();
    testDuplicateOldAndFreshOnceWithoutAllocations();
    testDeterministicTieAndFutureRejection();
    testHoldAnchoredToObservationTime();
    testBackwardTimeReset();
    std::printf("ALL TEAMMATE BALL TESTS PASSED\n");
    return 0;
}
