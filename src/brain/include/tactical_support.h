#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "tactical_types.h"

namespace tactics {
namespace support_detail {

constexpr double kFieldMargin = 0.6;
constexpr double kBallSeparation = 1.2;
constexpr double kOwnerSeparation = 1.0;
constexpr double kShotLaneClearance = 0.9;
constexpr double kApproachLaneClearance = 0.9;
constexpr double kBehindBall = 0.3;
constexpr std::size_t kMaxCandidates = 9;

struct Candidate {
    Point position;
    SupportRole role;
};

inline bool validRole(SupportRole role) {
    return role == SupportRole::Cover || role == SupportRole::Lateral ||
        role == SupportRole::Rebound;
}

inline bool validWorld(const World &world) {
    return world.ownerKnown && validField(world.field) && validBall(world.ball) &&
        finite(world.robot) && finite(world.owner) && world.obstacleCount <= kMaxObstacles &&
        std::abs(world.ball.position.x) <= world.field.length / 2.0 &&
        std::abs(world.ball.position.y) <= world.field.width / 2.0;
}

inline Point ownGoal(const Field &field) {
    return {-field.length / 2.0, 0.0};
}

inline Point attackingGoal(const Field &field) {
    return {field.length / 2.0, 0.0};
}

inline bool inPenaltyArea(Point p, const Field &field, bool attacking) {
    const double goalX = attacking ? field.length / 2.0 : -field.length / 2.0;
    const double boundary = attacking ? goalX - field.penaltyLength : goalX + field.penaltyLength;
    const bool alongField = attacking ? p.x >= boundary : p.x <= boundary;
    return alongField && std::abs(p.y) <= field.penaltyWidth / 2.0;
}

inline bool withinFieldMargin(Point p, const Field &field) {
    const double halfLength = field.length / 2.0;
    const double halfWidth = field.width / 2.0;
    return finite(p) && p.x >= -halfLength + kFieldMargin && p.x <= halfLength - kFieldMargin &&
        p.y >= -halfWidth + kFieldMargin && p.y <= halfWidth - kFieldMargin;
}

inline bool legalize(Point raw, const Field &field, Point &result) {
    if (!finite(raw)) return false;
    const double halfLength = field.length / 2.0;
    const double halfWidth = field.width / 2.0;
    result = {
        std::clamp(raw.x, -halfLength + kFieldMargin, halfLength - kFieldMargin),
        std::clamp(raw.y, -halfWidth + kFieldMargin, halfWidth - kFieldMargin)
    };
    return finite(result);
}

inline double cross(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool onSegment(Point a, Point b, Point p) {
    constexpr double epsilon = 1e-9;
    return p.x >= std::min(a.x, b.x) - epsilon && p.x <= std::max(a.x, b.x) + epsilon &&
        p.y >= std::min(a.y, b.y) - epsilon && p.y <= std::max(a.y, b.y) + epsilon;
}

inline bool segmentsIntersect(Point a, Point b, Point c, Point d) {
    const double abC = cross(a, b, c), abD = cross(a, b, d);
    const double cdA = cross(c, d, a), cdB = cross(c, d, b);
    if (((abC > 0.0 && abD < 0.0) || (abC < 0.0 && abD > 0.0)) &&
        ((cdA > 0.0 && cdB < 0.0) || (cdA < 0.0 && cdB > 0.0))) return true;
    return (std::abs(abC) <= 1e-9 && onSegment(a, b, c)) ||
        (std::abs(abD) <= 1e-9 && onSegment(a, b, d)) ||
        (std::abs(cdA) <= 1e-9 && onSegment(c, d, a)) ||
        (std::abs(cdB) <= 1e-9 && onSegment(c, d, b));
}

inline double segmentDistance(Point a, Point b, Point c, Point d) {
    if (segmentsIntersect(a, b, c, d)) return 0.0;
    return std::min({pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d),
        pointSegmentDistance(c, a, b), pointSegmentDistance(d, a, b)});
}

inline double endpointClearance(const World &world, Point target) {
    double clearance = std::numeric_limits<double>::infinity();
    const std::size_t count = std::min(world.obstacleCount, kMaxObstacles);
    for (std::size_t i = 0; i < count; ++i) {
        const Obstacle &obstacle = world.obstacles[i];
        if (!finite(obstacle.position) || !std::isfinite(obstacle.radius) || obstacle.radius < 0.0)
            return -1.0;
        clearance = std::min(clearance, distance(target, obstacle.position) - obstacle.radius);
    }
    return clearance;
}
inline bool obstaclePathClear(const World &world, Point from, Point target) {
    const Point owner = point(world.owner);
    const std::size_t count = std::min(world.obstacleCount, kMaxObstacles);
    for (std::size_t i = 0; i < count; ++i) {
        const Obstacle &obstacle = world.obstacles[i];
        if (!finite(obstacle.position) || !std::isfinite(obstacle.radius) || obstacle.radius < 0.0)
            return false;
        if (pointSegmentDistance(obstacle.position, from, target) > obstacle.radius) continue;

        const bool isOwner = distance(obstacle.position, owner) <= 1e-6;
        const Point fromOffset{from.x - obstacle.position.x, from.y - obstacle.position.y};
        const Point travel{target.x - from.x, target.y - from.y};
        const double outwardDot = fromOffset.x * travel.x + fromOffset.y * travel.y;
        if (!isOwner || distance(from, obstacle.position) > obstacle.radius ||
            distance(target, obstacle.position) <= obstacle.radius || outwardDot < -1e-9) return false;
    }
    return true;
}

inline bool crampedLateralException(const World &world, Point target) {
    const double halfLength = world.field.length / 2.0;
    const double xMin = -halfLength + kFieldMargin;
    const double xMax = halfLength - kFieldMargin;
    const double ballX = world.ball.position.x;
    const bool nearOwnEnd = world.ball.position.x <= 0.0 && ballX < xMin + 0.45;
    const bool nearOtherEnd = ballX > xMax - 0.45;
    const bool lateral = std::abs(target.y - world.ball.position.y) >= 1.2;
    return lateral && ((nearOwnEnd && target.x <= xMin + 1e-9) ||
        (nearOtherEnd && target.x >= xMax - 1e-9));
}

inline bool mayRebound(const World &world) {
    if (!world.ownerControlling || world.ball.position.x <= 0.0) return false;
    for (std::size_t i = 0; i < world.obstacleCount; ++i) {
        const auto &obstacle = world.obstacles[i];
        if (obstacle.opponent && distance(obstacle.position, world.ball.position) - obstacle.radius < 1.5) return false;
    }
    return true;
}

inline bool targetLegal(const World &world, Point target, SupportRole role) {
    if (!finite(target) || !withinFieldMargin(target, world.field) ||
        inPenaltyArea(target, world.field, false) || inPenaltyArea(target, world.field, true)) return false;

    const Point ball = world.ball.position;
    const Point owner = point(world.owner);
    if (distance(target, ball) < kBallSeparation || distance(target, owner) < kOwnerSeparation)
        return false;

    if (pointSegmentDistance(target, ball, attackingGoal(world.field)) < kShotLaneClearance)
        return false;
    if (pointSegmentDistance(target, owner, ball) < kApproachLaneClearance)
        return false;

    if (role == SupportRole::Rebound) {
        if (!mayRebound(world) || target.x < ball.x + 0.2 ||
            target.x > ball.x + 1.5 || std::abs(target.y - ball.y) < 1.2) return false;
    } else if (target.x > ball.x - kBehindBall && !crampedLateralException(world, target)) {
        return false;
    }

    const Point robot = point(world.robot);
    if (!obstaclePathClear(world, robot, target)) return false;
    if (endpointClearance(world, target) <= 0.0) return false;

    const Point opponentGoal = attackingGoal(world.field);
    const double startShotClearance = pointSegmentDistance(robot, ball, opponentGoal);
    if (startShotClearance >= kShotLaneClearance &&
        segmentDistance(robot, target, ball, opponentGoal) < kShotLaneClearance) return false;

    const double startApproachClearance = pointSegmentDistance(robot, owner, ball);
    if (startApproachClearance >= kApproachLaneClearance &&
        segmentDistance(robot, target, owner, ball) < kApproachLaneClearance) return false;
    return true;
}

inline bool highDanger(const World &world) {
    const Point goal = ownGoal(world.field);
    return world.ball.position.x <= 0.0 ||
        distance(world.ball.position, goal) < 0.35 * world.field.length;
}

inline double score(const World &world, Point target, SupportRole role) {
    const Point robot = point(world.robot);
    const Point ball = world.ball.position;
    const Point owner = point(world.owner);
    const Point ownGoalPoint = ownGoal(world.field);
    const Point opponentGoal = attackingGoal(world.field);
    const double walk = distance(robot, target);
    const double ballDistance = distance(ball, target);
    const double ownerDistance = distance(owner, target);
    const double openness = std::min(endpointClearance(world, target), 2.5);
    const double channelDistance = pointSegmentDistance(target, ball, ownGoalPoint);
    const bool danger = highDanger(world);

    double result = 0.55 * walk - 0.35 * openness;
    result -= 0.10 * std::min(std::max(0.0, ballDistance - kBallSeparation), 3.0);
    result -= 0.18 * std::min(std::max(0.0, ownerDistance - kOwnerSeparation), 3.0);
    result += (danger ? (role == SupportRole::Cover ? 1.15 : 1.65) : 0.28) * channelDistance;

    if (danger) result += role == SupportRole::Cover ? -0.45 : 0.3;
    else result += role == SupportRole::Lateral ? -0.25 : 0.15;
    if (role == SupportRole::Rebound) result -= 0.35;

    const double approachPathClearance = segmentDistance(robot, target, owner, ball);
    if (pointSegmentDistance(robot, owner, ball) < kApproachLaneClearance)
        result += 2.0 * std::max(0.0, kApproachLaneClearance - approachPathClearance);
    else
        result += 0.25 * std::max(0.0, 1.8 - pointSegmentDistance(target, owner, ball));

    const double shotPathClearance = segmentDistance(robot, target, ball, opponentGoal);
    if (pointSegmentDistance(robot, ball, opponentGoal) < kShotLaneClearance)
        result += 1.5 * std::max(0.0, kShotLaneClearance - shotPathClearance);
    return result;
}

inline void addCandidate(std::array<Candidate, kMaxCandidates> &candidates, std::size_t &count,
                         Point raw, SupportRole role, const World &world) {
    if (count >= candidates.size()) return;
    Point target;
    if (!legalize(raw, world.field, target) || !targetLegal(world, target, role)) return;
    candidates[count++] = Candidate{target, role};
}

} // namespace support_detail

inline SupportPlan chooseSupport(const World &world, const SupportPlan *previous = nullptr) {
    using namespace support_detail;
    SupportPlan invalid;
    if (!validWorld(world)) return invalid;

    const Point ball = world.ball.position;
    const Point goal = ownGoal(world.field);
    const double goalToBall = distance(goal, ball);
    if (!std::isfinite(goalToBall)) return invalid;
    const double behindX = goalToBall > 1e-9 ? (ball.x - goal.x) / goalToBall : 1.0;
    const double behindY = goalToBall > 1e-9 ? (ball.y - goal.y) / goalToBall : 0.0;

    std::array<Candidate, kMaxCandidates> candidates{};
    std::size_t count = 0;
    addCandidate(candidates, count,
        {ball.x - 2.1 * behindX, ball.y - 2.1 * behindY}, SupportRole::Cover, world);
    addCandidate(candidates, count, {ball.x - 1.7, ball.y - 1.8}, SupportRole::Lateral, world);
    addCandidate(candidates, count, {ball.x - 1.7, ball.y + 1.8}, SupportRole::Lateral, world);
    addCandidate(candidates, count, {ball.x - 0.9, ball.y - 2.35}, SupportRole::Lateral, world);
    addCandidate(candidates, count, {ball.x - 0.9, ball.y + 2.35}, SupportRole::Lateral, world);
    addCandidate(candidates, count, {ball.x - 2.8, ball.y - 1.35}, SupportRole::Cover, world);
    addCandidate(candidates, count, {ball.x - 2.8, ball.y + 1.35}, SupportRole::Cover, world);
    if (mayRebound(world)) {
        addCandidate(candidates, count, {ball.x + 0.85, ball.y - 2.0}, SupportRole::Rebound, world);
        addCandidate(candidates, count, {ball.x + 0.85, ball.y + 2.0}, SupportRole::Rebound, world);
    }
    if (count == 0) return invalid;

    std::size_t bestIndex = 0;
    double bestScore = score(world, candidates[0].position, candidates[0].role);
    for (std::size_t i = 1; i < count; ++i) {
        const double candidateScore = score(world, candidates[i].position, candidates[i].role);
        if (candidateScore < bestScore) {
            bestIndex = i;
            bestScore = candidateScore;
        }
    }

    Point selected = candidates[bestIndex].position;
    SupportRole selectedRole = candidates[bestIndex].role;
    if (previous != nullptr && previous->valid && finite(previous->target) &&
        validRole(previous->role) && targetLegal(world, point(previous->target), previous->role)) {
        const Point oldTarget = point(previous->target);
        const double oldScore = score(world, oldTarget, previous->role);
        constexpr double kPreviousScoreMargin = 1.5;
        constexpr double kMaxReferenceShift = 0.4;
        if (std::isfinite(oldScore) && oldScore <= bestScore + kPreviousScoreMargin &&
            distance(previous->referenceBall, ball) <= kMaxReferenceShift &&
            distance(previous->referenceOwner, point(world.owner)) <= kMaxReferenceShift) {
            selected = oldTarget;
            selectedRole = previous->role;
        }
    }

    SupportPlan result;
    result.valid = true;
    const bool held = previous != nullptr && previous->valid &&
        distance(selected, point(previous->target)) < 1e-9 && selectedRole == previous->role;
    result.referenceBall = held ? previous->referenceBall : ball;
    result.referenceOwner = held ? previous->referenceOwner : point(world.owner);
    result.target = {selected.x, selected.y, wrap(angle(selected, ball))};
    result.role = selectedRole;
    if (!finite(result.target)) return invalid;
    return result;
}

} // namespace tactics
