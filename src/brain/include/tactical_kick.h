#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "tactical_types.h"

namespace tactics {
namespace kick_detail {

constexpr std::size_t kSampleCount = 25;
constexpr double kGoalPostMargin = 0.2;
constexpr double kMaxGroundProgress = 1.5;
constexpr double kMinimumRouteClearance = 0.02;
constexpr double kFieldEdgeReserve = 0.02;
constexpr double kMinimumCarryLength = 0.15;
constexpr double kAlignmentWeight = 0.12;
constexpr double kHighPressureSeconds = 0.9;
constexpr double kKickExecutionReserveSeconds = 0.35;
constexpr double kMaxReportedClearance = 10.0;
constexpr double kCarryHalfAngle = 75.0 * kPi / 180.0;
constexpr double kGoalLineEpsilon = 0.01;

struct RouteSample {
    bool clear = false;
    double direction = 0.0;
    double clearance = -1.0;
    Point target;
};

struct Lane {
    bool valid = false;
    double direction = 0.0;
    double windowRadians = 0.0;
    double clearance = -1.0;
    double alignmentCost = 0.0;
    Point target;
};

inline double alignmentCost(const World &world, double direction)
{
    const double headingCost = std::abs(wrap(direction - world.robot.theta));
    const Point robotPosition = point(world.robot);
    const double robotBallDistance = distance(robotPosition, world.ball.position);
    if (robotBallDistance <= 0.1) return headingCost;

    // A robot already behind the ball and facing the intended lane needs less
    // repositioning than one whose ball-to-robot line points across the lane.
    const double approachDirection = angle(robotPosition, world.ball.position);
    const double approachCost = std::abs(wrap(direction - approachDirection));
    return 0.7 * headingCost + 0.3 * approachCost;
}

inline double reportedClearance(double clearance)
{
    return std::isfinite(clearance) ? std::min(clearance, kMaxReportedClearance)
                                    : kMaxReportedClearance;
}

inline bool observedRoute(const World &world, Point target)
{
    if (!world.viewReliable || !world.goalObserved) return false;
    double orientation = 0.0;
    for (std::size_t i = 0; i < world.viewFootprint.size(); ++i) {
        const Point a = world.viewFootprint[i];
        const Point b = world.viewFootprint[(i + 1) % 4];
        const Point c = world.viewFootprint[(i + 2) % 4];
        if (!finite(a)) return false;
        const double turn = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if (!std::isfinite(turn) || std::abs(turn) <= 1e-9) return false;
        if (i == 0) orientation = turn;
        else if (orientation * turn <= 0.0) return false;
        for (Point p : {world.ball.position, target}) {
            const double side = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
            if (!std::isfinite(side) || side * orientation < -1e-9) return false;
        }
    }
    return true;
}

inline Point goalPlaneTarget(const World &world, double direction)
{
    const double goalX = world.field.length / 2.0;
    const double dx = goalX - world.ball.position.x;
    return {goalX, world.ball.position.y + std::tan(direction) * dx};
}

inline bool insideField(const World &world, Point p)
{
    const double halfLength = world.field.length / 2.0;
    const double halfWidth = world.field.width / 2.0;
    return finite(p) && p.x >= -halfLength - 1e-9 && p.x <= halfLength + 1e-9 &&
        p.y >= -halfWidth - 1e-9 && p.y <= halfWidth + 1e-9;
}

inline double routeLengthToFieldEdge(const World &world, double direction)
{
    const Point origin = world.ball.position;
    const double dx = std::cos(direction);
    const double dy = std::sin(direction);
    const double halfLength = world.field.length / 2.0;
    const double halfWidth = world.field.width / 2.0;
    double limit = std::numeric_limits<double>::infinity();

    if (dx > 1e-12) limit = std::min(limit, (halfLength - origin.x) / dx);
    else if (dx < -1e-12) limit = std::min(limit, (-halfLength - origin.x) / dx);
    if (dy > 1e-12) limit = std::min(limit, (halfWidth - origin.y) / dy);
    else if (dy < -1e-12) limit = std::min(limit, (-halfWidth - origin.y) / dy);
    return limit;
}

inline RouteSample evaluateRoute(const World &world, double direction, double requestedLength)
{
    RouteSample sample;
    sample.direction = direction;
    const double edgeLength = routeLengthToFieldEdge(world, direction);
    if (!std::isfinite(edgeLength) || edgeLength <= kFieldEdgeReserve) return sample;

    const double travel = std::min(requestedLength, edgeLength - kFieldEdgeReserve);
    if (!std::isfinite(travel) || travel < kMinimumCarryLength) return sample;
    sample.target = {world.ball.position.x + travel * std::cos(direction),
                     world.ball.position.y + travel * std::sin(direction)};
    if (!insideField(world, sample.target)) return sample;

    sample.clearance = segmentClearance(world, world.ball.position, sample.target);
    sample.clear = !std::isnan(sample.clearance) &&
        sample.clearance >= kMinimumRouteClearance;
    // With no nearby disk the common helper returns +infinity; a route's
    // reported clearance is deliberately saturated to a finite meter value.
    if (sample.clear && !std::isfinite(sample.clearance)) {
        sample.clearance = kMaxReportedClearance;
    }
    return sample;
}

inline Lane widestLane(const World &world, const std::array<RouteSample, kSampleCount> &samples,
                       double totalAngularRange)
{
    Lane best;
    const double sampleStep = totalAngularRange / static_cast<double>(kSampleCount - 1);
    for (std::size_t begin = 0; begin < kSampleCount;) {
        if (!samples[begin].clear) {
            ++begin;
            continue;
        }
        std::size_t end = begin;
        while (end + 1 < kSampleCount && samples[end + 1].clear) ++end;

        const std::size_t center = begin + (end - begin) / 2;
        const double width = std::min(totalAngularRange,
            static_cast<double>(end - begin + 1) * sampleStep);
        const double alignment = alignmentCost(world, samples[center].direction);
        const double score = width - kAlignmentWeight * alignment;
        const double bestScore = best.windowRadians - kAlignmentWeight * best.alignmentCost;
        if (!best.valid || score > bestScore + 1e-12 ||
            (std::abs(score - bestScore) <= 1e-12 &&
             samples[center].clearance > best.clearance)) {
            best.valid = true;
            best.direction = samples[center].direction;
            best.windowRadians = width;
            best.clearance = samples[center].clearance;
            best.alignmentCost = alignment;
            best.target = samples[center].target;
        }
        begin = end + 1;
    }
    return best;
}

inline double opponentArrival(const World &world)
{
    double arrival = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < world.obstacleCount; ++i) {
        const Obstacle &obstacle = world.obstacles[i];
        if (!obstacle.opponent) continue;
        const double distanceToBall = distance(obstacle.position, world.ball.position);
        arrival = std::min(arrival, std::max(0.0, distanceToBall - obstacle.radius) / 1.2);
    }
    return arrival;
}

inline bool validInput(const World &world, double minimumWindowRadians)
{
    if (!validField(world.field) || !validLimits(world.limits) || !validBall(world.ball) ||
        !finite(world.robot) || !std::isfinite(minimumWindowRadians) ||
        minimumWindowRadians < 0.0 || minimumWindowRadians > kPi ||
        world.obstacleCount > kMaxObstacles || (world.ownerKnown && !finite(world.owner)) ||
        (world.ball.velocityKnown && !finite(world.ball.velocity))) {
        return false;
    }

    const double halfLength = world.field.length / 2.0;
    const double halfWidth = world.field.width / 2.0;
    const Point ball = world.ball.position;
    // A ball behind its own goal, over a touchline, or already across the
    // opponent goal plane has no legal forward target in this planner.
    if (ball.x < -halfLength || ball.x >= halfLength - kGoalLineEpsilon ||
        ball.y < -halfWidth || ball.y > halfWidth) {
        return false;
    }
    for (std::size_t i = 0; i < world.obstacleCount; ++i) {
        const Obstacle &obstacle = world.obstacles[i];
        if (!finite(obstacle.position) || !std::isfinite(obstacle.radius) || obstacle.radius < 0.0) {
            return false;
        }
    }
    return true;
}

inline bool inOwnPenaltyArea(const World &world)
{
    const double ownGoalX = -world.field.length / 2.0;
    return world.ball.position.x <= ownGoalX + world.field.penaltyLength &&
        std::abs(world.ball.position.y) <= world.field.penaltyWidth / 2.0;
}

inline Lane chooseGoalLane(const World &world)
{
    const double goalX = world.field.length / 2.0;
    const double halfWidth = world.field.goalWidth / 2.0 - kGoalPostMargin;
    const double lower = angle(world.ball.position, {goalX, -halfWidth});
    const double upper = angle(world.ball.position, {goalX, halfWidth});
    struct Interval { double lower, upper; };
    std::array<Interval, kMaxObstacles + 1> windows{};
    std::size_t count = 1;
    windows[0] = {lower, upper};
    for (std::size_t obstacleIndex = 0; obstacleIndex < world.obstacleCount; ++obstacleIndex) {
        const auto &obstacle = world.obstacles[obstacleIndex];
        const double radius = obstacle.radius + kMinimumRouteClearance;
        if (obstacle.position.x - radius >= goalX || obstacle.position.x + radius <= world.ball.position.x) continue;
        const double range = distance(obstacle.position, world.ball.position);
        if (range <= radius) return Lane{};
        const double center = angle(world.ball.position, obstacle.position);
        const double halfAngle = std::asin(std::clamp(radius / range, 0.0, 1.0));
        const double blockedLower = center - halfAngle;
        const double blockedUpper = center + halfAngle;
        // Each disk can split at most one disjoint window; capacity is N+1.
        for (std::size_t i = count; i-- > 0;) {
            const auto window = windows[i];
            if (blockedUpper <= window.lower || blockedLower >= window.upper) continue;
            if (blockedLower <= window.lower && blockedUpper >= window.upper) {
                std::move(windows.begin() + i + 1, windows.begin() + count, windows.begin() + i);
                --count;
            } else if (blockedLower <= window.lower) {
                windows[i].lower = blockedUpper;
            } else if (blockedUpper >= window.upper) {
                windows[i].upper = blockedLower;
            } else {
                std::move_backward(windows.begin() + i + 1, windows.begin() + count, windows.begin() + count + 1);
                windows[i] = {window.lower, blockedLower};
                windows[i + 1] = {blockedUpper, window.upper};
                ++count;
            }
        }
    }
    Lane best;
    double bestScore = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < count; ++i) {
        const double width = windows[i].upper - windows[i].lower;
        const double direction = (windows[i].upper + windows[i].lower) / 2.0;
        const Point target = goalPlaneTarget(world, direction);
        const double clearance = segmentClearance(world, world.ball.position, target);
        if (width <= 0.0 || !insideField(world, target) || clearance < kMinimumRouteClearance) continue;
        const double alignment = alignmentCost(world, direction);
        const double score = width - kAlignmentWeight * alignment;
        if (score > bestScore) {
            best = {true, direction, width, reportedClearance(clearance), alignment, target};
            bestScore = score;
        }
    }
    return best;
}

inline Lane chooseForwardLane(const World &world)
{
    std::array<RouteSample, kSampleCount> samples{};
    for (std::size_t i = 0; i < kSampleCount; ++i) {
        const double fraction = static_cast<double>(i) / static_cast<double>(kSampleCount - 1);
        const double direction = -kCarryHalfAngle + fraction * (2.0 * kCarryHalfAngle);
        samples[i] = evaluateRoute(world, direction, kMaxGroundProgress);
    }
    return widestLane(world, samples, 2.0 * kCarryHalfAngle);
}

inline KickPlan makePlan(Action action, const World &world, const Lane &lane, double arrival)
{
    KickPlan plan;
    if (!lane.valid || !insideField(world, lane.target)) return plan;
    plan.valid = true;
    plan.action = action;
    plan.target = lane.target;
    plan.direction = wrap(angle(world.ball.position, lane.target));
    plan.clearance = reportedClearance(lane.clearance);
    plan.windowRadians = lane.windowRadians;
    plan.opponentArrivalSeconds = arrival;
    plan.visualKickAllowed = action == Action::Shoot && observedRoute(world, lane.target);
    return plan;
}

} // namespace kick_detail

inline KickPlan chooseKick(const World &world, double minimumWindowRadians = 0.18)
{
    using namespace kick_detail;
    if (!validInput(world, minimumWindowRadians)) return KickPlan{};

    const double arrival = opponentArrival(world);
    const Lane goalLane = chooseGoalLane(world);
    const bool ownGoalDanger = inOwnPenaltyArea(world);
    const double alignmentSeconds = goalLane.valid
        ? goalLane.alignmentCost / world.limits.turn
        : 0.0;
    const bool urgentPressure = std::isfinite(arrival) &&
        (arrival <= kHighPressureSeconds ||
         (goalLane.valid && arrival <= alignmentSeconds + kKickExecutionReserveSeconds));

    if (ownGoalDanger || urgentPressure) {
        const Lane clearLane = chooseForwardLane(world);
        return makePlan(Action::Clear, world, clearLane, arrival);
    }
    if (goalLane.valid && goalLane.windowRadians + 1e-12 >= minimumWindowRadians) {
        return makePlan(Action::Shoot, world, goalLane, arrival);
    }

    const Lane carryLane = chooseForwardLane(world);
    return makePlan(Action::Carry, world, carryLane, arrival);
}

} // namespace tactics
