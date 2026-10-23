#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include "tactical_timing.h"
#include "team_strategy.h"

int main() {
    tactics::World world;
    world.field = {14.16, 9.22, 2.6, 3.24, 6.192};
    world.limits = {0.9, 0.2, 1.0};
    world.ball.known = true;
    world.ball.position = {0, 0};
    world.ball.confidence = 90;
    const auto aligned = tactics::estimateTouch(world, {-1.2, 0, 0}, 0);
    const auto sideways = tactics::estimateTouch(world, {-0.2, 0.6, 0}, 0);
    assert(aligned.valid && aligned.reachable && sideways.valid && sideways.reachable);
    assert(aligned.seconds < sideways.seconds); // Farther but already behind the ball.
    std::array<team_strategy::Candidate, team_strategy::kCapacity> players{};
    players[0] = {1, true, false, false, sideways.seconds};
    players[1] = {2, true, false, false, aligned.seconds};
    assert(team_strategy::owner(players, 3, 0.3) == 2);
    std::printf("touch timing: aligned=%.3fs sideways=%.3fs owner=%d\n",
        aligned.seconds, sideways.seconds, team_strategy::owner(players, 3, 0.3));
    world.ball.position = {0.6, 0};
    world.ball.velocity = {1, 0};
    world.ball.velocityKnown = true;
    const auto chasing = tactics::estimateTouch(world, {0, 0, 0}, 0);
    const auto intercepting = tactics::estimateTouch(world, {1.4, 0, 0}, 0);
    assert(chasing.valid && !chasing.reachable && chasing.seconds == 100);
    assert(intercepting.valid && intercepting.reachable && intercepting.seconds <= 6);
    std::printf("rolling ball: behind reachable=%d, ahead contact=%.3fs\n", chasing.reachable, intercepting.seconds);
    world.ball.position = {0, 0};
    world.ball.velocityKnown = false;
    world.obstacleCount = 1;
    world.obstacles[0] = {{-0.35, 0}, 0.45, true};
    assert(!tactics::estimateTouch(world, {-1, 0, 0}, 0).reachable);
    world.obstacles[0] = {{-1, 0}, 0.45, false};
    assert(tactics::estimateTouch(world, {-1, 0, 0}, 0).reachable); // Self disk is not a blocker.
    world.obstacleCount = 0;
    world.ball.position = {-7.0, 0};
    assert(!tactics::estimateTouch(world, {-6, 0, 0}, 0).reachable);
    world.ball.position = {0, 0};
    world.ball.confidence = std::numeric_limits<double>::quiet_NaN();
    assert(!tactics::estimateTouch(world, {-1, 0, 0}, 0).valid);
    world.ball.confidence = 90;
    world.limits.lateral = 0;
    assert(!tactics::estimateTouch(world, {-1, 0, 0}, 0).valid);

    std::array<tactics::BallSample, 11> samples{};
    tactics::Ball first, second;
    first.position = {1, 2}; first.known = true; first.confidence = 80; first.ageSeconds = 0.1;
    second = first; second.position = {3, 4};
    samples[0] = {2, first, true}; samples[1] = {1, second, true};
    assert(tactics::selectTeamBall(samples, 3).position.x == 3);
    std::swap(samples[0], samples[1]);
    assert(tactics::selectTeamBall(samples, 3).position.x == 3);
    samples[0].ball.ageSeconds = -0.1;
    assert(tactics::selectTeamBall(samples, 3).position.x == 1);
    samples[1].ball.ageSeconds = 0.651;
    assert(!tactics::selectTeamBall(samples, 3).known);
    std::puts("tactical_timing_test: heading, interception horizon, obstacles, source and invalid boundaries passed");
}
