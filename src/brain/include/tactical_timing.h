#pragma once

#include "tactical_types.h"

namespace tactics {
struct TouchEstimate {
    bool valid = false;
    bool reachable = false;
    double seconds = 100.0;
    Point contactTarget;
};
struct BallSample {
    int playerId = 0;
    Ball ball;
    bool available = false;
};

inline Ball selectTeamBall(const std::array<BallSample, 11> &samples, int count) {
    const BallSample *best = nullptr;
    for (int i = 0; i < std::clamp(count, 0, 11); ++i) {
        const auto &sample = samples[i];
        if (!sample.available || !validBall(sample.ball) || sample.ball.ageSeconds > 0.65 ||
            sample.ball.confidence < 50.0 || sample.playerId < 1 || sample.playerId > count) continue;
        if (!best || sample.ball.confidence > best->ball.confidence ||
            (sample.ball.confidence == best->ball.confidence && sample.playerId < best->playerId)) best = &sample;
    }
    return best ? best->ball : Ball{};
}

inline double travelSeconds(Pose robot, Point target, double finalHeading, Limits limits) {
    if (!finite(robot) || !finite(target) || !std::isfinite(finalHeading) || !validLimits(limits))
        return std::numeric_limits<double>::infinity();
    const double dx = target.x - robot.x, dy = target.y - robot.y;
    const double rx = dx * std::cos(robot.theta) + dy * std::sin(robot.theta);
    const double ry = -dx * std::sin(robot.theta) + dy * std::cos(robot.theta);
    const double heading = std::abs(wrap(finalHeading - robot.theta)) / limits.turn;
    // Compare slow sidestepping with turn-walk-turn using the configured gait limits.
    const double direct = std::max(std::abs(rx) / limits.forward, std::abs(ry) / limits.lateral) + heading;
    const double direction = distance(point(robot), target) > 1e-6 ? angle(point(robot), target) : robot.theta;
    const double turning = std::abs(wrap(direction - robot.theta)) / limits.turn +
        distance(point(robot), target) / limits.forward + std::abs(wrap(finalHeading - direction)) / limits.turn;
    return std::min(direct, turning);
}

inline TouchEstimate estimateTouch(const World &world, Pose robot, double kickDirection) {
    TouchEstimate result;
    if (!validField(world.field) || !validLimits(world.limits) || !validBall(world.ball) ||
        !finite(robot) || !std::isfinite(kickDirection) || world.obstacleCount > kMaxObstacles) return result;
    const bool moving = world.ball.velocityKnown && world.ball.ageSeconds <= 0.5;
    if (moving && (!finite(world.ball.velocity) || std::hypot(world.ball.velocity.x, world.ball.velocity.y) > 5.0))
        return result;
    result.valid = true;
    constexpr double stanceDistance = 0.35;
    constexpr double horizon = 6.0;
    const int samples = moving ? 61 : 1;
    for (int sample = 0; sample < samples; ++sample) {
        const double time = sample * 0.1;
        const Point ball{
            world.ball.position.x + (moving ? world.ball.velocity.x * time : 0.0),
            world.ball.position.y + (moving ? world.ball.velocity.y * time : 0.0)};
        const Point target{ball.x - stanceDistance * std::cos(kickDirection),
                           ball.y - stanceDistance * std::sin(kickDirection)};
        // Do not advertise a contact stance outside the field as reachable.
        if (std::abs(target.x) > world.field.length / 2.0 - 0.15 ||
            std::abs(target.y) > world.field.width / 2.0 - 0.15 ||
            std::abs(ball.x) > world.field.length / 2.0 || std::abs(ball.y) > world.field.width / 2.0) continue;
        double seconds = travelSeconds(robot, target, kickDirection, world.limits) + 0.2;
        const double ballPath = pointSegmentDistance(ball, point(robot), target);
        if (ballPath < 0.22 && distance(point(robot), ball) > 0.45) {
            seconds += 0.6 / world.limits.lateral; // Need to get around the ball, not walk through it.
        }
        for (std::size_t i = 0; i < world.obstacleCount; ++i) {
            const auto &obstacle = world.obstacles[i];
            if (!finite(obstacle.position) || !std::isfinite(obstacle.radius) || obstacle.radius < 0.0) return TouchEstimate{};
            // The shared world may contain this candidate's own reported footprint.
            if (!obstacle.opponent && distance(obstacle.position, point(robot)) < 0.05) continue;
            if (distance(obstacle.position, target) <= obstacle.radius + 0.1) { seconds = 100.0; break; }
            if (pointSegmentDistance(obstacle.position, point(robot), target) < obstacle.radius + 0.15)
                seconds += (2.0 * obstacle.radius + 0.3) / world.limits.forward + 0.5;
        }
        seconds += world.ball.ageSeconds + (100.0 - world.ball.confidence) * 0.005;
        if (!moving || seconds <= time) {
            result.seconds = moving ? time : std::min(seconds, 100.0);
            result.reachable = std::isfinite(seconds) && seconds < 100.0 && (!moving || time <= horizon);
            result.contactTarget = target;
            return result;
        }
    }
    // A root beyond the horizon is not an interception inside the horizon.
    return result;
}
} // namespace tactics
