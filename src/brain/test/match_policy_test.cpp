#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "match_policy.h"
#include "team_strategy.h"

namespace {
void restartAndStopTransitions() {
    match_policy::Input input;
    input.version = 20;
    input.primary = 3;
    input.phase = 2;
    input.setPlay = 6;
    input.penalized = false;
    input.ownKickoff = true;
    auto state = match_policy::interpret(input);
    assert(state.valid && state.phase == match_policy::Phase::ExtraTime);
    assert(state.restart == match_policy::Restart::Corner);
    assert(!match_policy::canMove(state, true, 0, 2000, true));
    assert(!match_policy::canKick(state, true, 0, 2000, true));
    input.stopped = false;
    state = match_policy::interpret(input);
    assert(match_policy::canMove(state, true, 100, 2000, true));
    assert(match_policy::canKick(state, true, 100, 2000, true));
    input.ownKickoff = false;
    state = match_policy::interpret(input);
    assert(match_policy::canMove(state, true, 100, 2000, true));
    assert(!match_policy::canKick(state, true, 100, 2000, true));
    input.primary = 1;
    input.setPlay = 0;
    state = match_policy::interpret(input);
    assert(match_policy::canMove(state, true, 100, 2000, true));
    assert(!match_policy::canKick(state, true, 100, 2000, true));
    input.primary = 2;
    state = match_policy::interpret(input);
    assert(!match_policy::canMove(state, true, 100, 2000, true));
}

void authorizationBoundaries() {
    match_policy::Input input;
    input.version = 20;
    input.primary = 3;
    input.stopped = false;
    input.penalized = false;
    assert(match_policy::newerPacket(0, 255));
    assert(match_policy::newerPacket(12, 11));
    assert(!match_policy::newerPacket(11, 11));
    assert(!match_policy::newerPacket(10, 11));
    assert(!match_policy::newerPacket(139, 11));
    auto state = match_policy::interpret(input);
    assert(!match_policy::canMove(state, false, 0, 2000, true));
    assert(match_policy::canMove(state, false, 0, 2000, false));
    assert(match_policy::canMove(state, true, 2000, 2000, true));
    assert(!match_policy::canMove(state, true, 2000.01, 2000, true));
    assert(!match_policy::canMove(state, true, -0.1, 2000, true));
    assert(!match_policy::canMove(state, true, NAN, 2000, true));
    assert(!match_policy::canMove(state, true, 1, NAN, true));
    input.penalized = true;
    assert(!match_policy::canMove(match_policy::interpret(input), true, 1, 2000, true));
    input.penalized = false;
    input.phase = 3;
    assert(!match_policy::canMove(match_policy::interpret(input), true, 1, 2000, true));
    input.phase = 1;
    input.ownKickoff = false;
    assert(!match_policy::canKick(match_policy::interpret(input), true, 1, 2000, true));
    input.ownKickoff = true;
    assert(match_policy::canKick(match_policy::interpret(input), true, 1, 2000, true));
    input.phase = 0;
    input.ownKickoff = false;
    input.secondarySeconds = 5;
    assert(!match_policy::canKick(match_policy::interpret(input), true, 1, 2000, true));
    input.secondarySeconds = 0;
    assert(match_policy::canKick(match_policy::interpret(input), true, 1, 2000, true));
    input.setPlay = 7;
    assert(!match_policy::interpret(input).valid);
    input.version = 21;
    assert(!match_policy::interpret(input).valid);

    input = {};
    input.version = 12;
    input.primary = 3;
    input.legacySecondary = 8;
    input.legacyStage = 1;
    input.ownSetPlay = true;
    input.stopped = false;
    input.penalized = false;
    state = match_policy::interpret(input);
    assert(state.restart == match_policy::Restart::GoalKick);
    assert(match_policy::canMove(state, true, 0, 2000, true));
    assert(!match_policy::canKick(state, true, 0, 2000, true));
    input.legacyStage = 2;
    assert(!match_policy::canMove(match_policy::interpret(input), true, 0, 2000, true));
    input.primary = 0;
    input.legacySecondary = 0;
    input.stopped = true;
    assert(match_policy::interpret(input).stopped);
}

void teamRoleAndOwnerTransitions() {
    std::array<bool, team_strategy::kCapacity> eligible{};
    eligible[0] = eligible[1] = eligible[2] = true;
    eligible[8] = true; // Unused storage must not create a participating player.
    assert(team_strategy::goalkeeper(eligible, 3, 3, true) == 3);
    eligible[2] = false;
    assert(team_strategy::goalkeeper(eligible, 3, 3, true) == 1);
    assert(team_strategy::goalkeeper(eligible, 3, 3, false) == 0);
    assert(team_strategy::fallbackOwner(eligible, 3, 1) == 2);
    eligible[0] = eligible[1] = false;
    assert(team_strategy::goalkeeper(eligible, 3, 3, true) == 0);
    assert(team_strategy::fallbackOwner(eligible, 3, 0) == 0);

    std::array<team_strategy::Candidate, team_strategy::kCapacity> players{};
    players[0] = {1, true, false, false, 2.01};
    players[1] = {2, true, false, false, 2.00};
    players[2] = {3, false, false, false, 0.1};
    assert(team_strategy::owner(players, 3, 0.3) == 1);
    std::swap(players[0], players[1]);
    assert(team_strategy::owner(players, 3, 0.3) == 1);
    players[0].claimsOwner = true;
    assert(team_strategy::owner(players, 3, 0.3) == 2);
    players[0].cost = 2.5;
    assert(team_strategy::owner(players, 3, 0.3) == 1);
    players[0].visualKick = true;
    assert(team_strategy::owner(players, 3, 0.3) == 2);
    players[1].visualKick = true;
    assert(team_strategy::owner(players, 3, 0.3) == 1);
    players[1].available = false;
    assert(team_strategy::owner(players, 3, 0.3) == 2);
    players[0].cost = std::numeric_limits<double>::quiet_NaN();
    assert(team_strategy::owner(players, 3, 0.3) == 0);
}
}

int main() {
    restartAndStopTransitions();
    authorizationBoundaries();
    teamRoleAndOwnerTransitions();
    std::puts("match_policy_test: restart, stop, penalty, roster and owner boundaries passed");
}
