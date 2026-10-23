#include <cstddef>
#include <cassert>
#include <cmath>
#include <limits>

#include "tactical_kick.h"

namespace {

using tactics::Action;
using tactics::KickPlan;
using tactics::Obstacle;
using tactics::Point;
using tactics::World;
using tactics::kMaxObstacles;

World makeWorld()
{
    World world;
    world.field = {9.0, 6.0, 1.5, 1.0, 2.5};
    world.robot = {-0.2, 0.0, 0.0};
    world.ball.position = {0.0, 0.0};
    world.ball.known = true;
    world.ball.confidence = 100.0;
    world.goalObserved = true;
    world.viewReliable = true;
    world.viewFootprint = {{{-2.0, -4.0}, {6.0, -4.0}, {6.0, 4.0}, {-2.0, 4.0}}};
    return world;
}

void assertInvalidHold(const KickPlan &plan)
{
    assert(!plan.valid);
    assert(plan.action == Action::Hold);
    assert(!plan.visualKickAllowed);
}

bool insideField(const World &world, Point p)
{
    return p.x >= -world.field.length / 2.0 && p.x <= world.field.length / 2.0 &&
        p.y >= -world.field.width / 2.0 && p.y <= world.field.width / 2.0;
}

void assertSafeShotWithoutVisual(const World &world)
{
    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Shoot);
    assert(!plan.visualKickAllowed);
    assert(plan.target.x > world.ball.position.x);
    assert(plan.clearance >= 0.02);
    assert(tactics::segmentClearance(world, world.ball.position, plan.target) >= 0.02);
}

void setGoalGapWidth(World &world, double gapWidth)
{
    const double goalX = world.field.length / 2.0;
    const double goalHalfWidth = world.field.goalWidth / 2.0 - 0.2;
    const double lower = tactics::angle(world.ball.position, {goalX, -goalHalfWidth});
    const double blockerHalfAngle = 0.08;
    const double blockerDistance = 2.0;
    const double blockerAngle = lower + gapWidth + blockerHalfAngle;
    world.obstacles[0] = {
        {world.ball.position.x + blockerDistance * std::cos(blockerAngle),
         world.ball.position.y + blockerDistance * std::sin(blockerAngle)},
        blockerDistance * std::sin(blockerHalfAngle) - 0.02,
        false};
    world.obstacleCount = 1;
}

void openGoalUsesBallOriginAndEvidenceGatesVisualKick()
{
    World world = makeWorld();
    world.ball.position = {0.0, 0.8};
    world.robot = {1.0, -1.0, 0.0};

    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Shoot);
    assert(plan.visualKickAllowed);
    assert(std::abs(plan.target.x - world.field.length / 2.0) < 1e-9);
    assert(std::abs(plan.target.y) <= world.field.goalWidth / 2.0 - 0.2 + 1e-9);
    assert(std::abs(tactics::wrap(plan.direction - tactics::angle(world.ball.position, plan.target))) < 1e-9);
    assert(std::abs(tactics::wrap(plan.direction - tactics::angle(tactics::point(world.robot), plan.target))) > 0.1);

    world.goalObserved = false;
    assert(!tactics::chooseKick(world).visualKickAllowed);
    world.goalObserved = true;
    world.viewReliable = false;
    assert(!tactics::chooseKick(world).visualKickAllowed);
}

void centerBlockerUsesOnlyClearRoutes()
{
    World world = makeWorld();
    world.obstacles[0] = {{2.0, 0.0}, 0.4, false};
    world.obstacleCount = 1;

    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Carry);
    assert(!plan.visualKickAllowed);
    assert(plan.target.x > world.ball.position.x);
    assert(tactics::segmentClearance(world, world.ball.position, plan.target) >= 0.02);
    assert(insideField(world, plan.target));
}

void leftBlockerSelectsRightGoalLane()
{
    World world = makeWorld();
    world.obstacles[0] = {{2.0, 0.35}, 0.2, false};
    world.obstacleCount = 1;

    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Shoot);
    assert(plan.direction < 0.0);
    assert(plan.windowRadians >= 0.18);
    assert(plan.clearance >= 0.02);
}

void tooNarrowWindowSuppressesVisualKick()
{
    World world = makeWorld();
    const KickPlan plan = tactics::chooseKick(world, 0.25);
    assert(plan.valid && plan.action == Action::Carry);
    assert(!plan.visualKickAllowed);
}

void exactGoalWindowThresholdIsNotInflatedBySampling()
{
    World exact = makeWorld();
    setGoalGapWidth(exact, 0.18);
    const KickPlan atThreshold = tactics::chooseKick(exact, 0.18);
    assert(atThreshold.valid && atThreshold.action == Action::Shoot);
    assert(std::abs(atThreshold.windowRadians - 0.18) < 1e-10);

    World justBelow = makeWorld();
    setGoalGapWidth(justBelow, 0.179);
    const KickPlan rejected = tactics::chooseKick(justBelow, 0.18);
    assert(rejected.valid && rejected.action == Action::Carry);
    assert(!rejected.visualKickAllowed);

    const KickPlan accepted = tactics::chooseKick(justBelow, 0.178);
    assert(accepted.valid && accepted.action == Action::Shoot);
    assert(std::abs(accepted.windowRadians - 0.179) < 1e-10);
}

void multipleBlockersLeaveOnlySafeGroundRoutes()
{
    World world = makeWorld();
    world.obstacles[0] = {{2.0, -0.15}, 0.35, false};
    world.obstacles[1] = {{2.0, 0.15}, 0.35, false};
    world.obstacleCount = 2;

    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Carry);
    assert(!plan.visualKickAllowed);
    assert(plan.clearance >= 0.02);
    assert(tactics::segmentClearance(world, world.ball.position, plan.target) >= 0.02);
    assert(insideField(world, plan.target));
}

void fullObstacleCapacityStillFindsSafeGroundRoute()
{
    World world = makeWorld();
    world.field.goalWidth = 5.0;
    for (std::size_t i = 0; i < kMaxObstacles; ++i) {
        world.obstacles[i] = {{4.0, -1.9 + 3.8 * i / (kMaxObstacles - 1)}, 0.01, false};
    }
    world.obstacleCount = kMaxObstacles;

    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Carry);
    assert(!plan.visualKickAllowed);
    assert(plan.clearance >= 0.02);
    assert(tactics::segmentClearance(world, world.ball.position, plan.target) >= 0.02);
    assert(insideField(world, plan.target));
}

void invalidStaleAndDegenerateFootprintsDisableOnlyVisualKick()
{
    World invalid = makeWorld();
    invalid.viewFootprint[0].x = std::numeric_limits<double>::quiet_NaN();
    assertSafeShotWithoutVisual(invalid);

    World stale = makeWorld();
    stale.viewReliable = false;
    assertSafeShotWithoutVisual(stale);

    World degenerate = makeWorld();
    degenerate.viewFootprint = {{{-2.0, -4.0}, {-2.0, -1.0}, {-2.0, 1.0}, {-2.0, 4.0}}};
    assertSafeShotWithoutVisual(degenerate);

    World targetOutsideView = makeWorld();
    targetOutsideView.viewFootprint = {{{-2.0, -4.0}, {4.0, -4.0}, {4.0, 4.0}, {-2.0, 4.0}}};
    assertSafeShotWithoutVisual(targetOutsideView);
}

void ownPenaltyDangerClearsOnlyIntoLegalForwardSpace()
{
    World world = makeWorld();
    world.ball.position = {-4.0, 0.4};

    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Clear);
    assert(plan.target.x > world.ball.position.x);
    assert(std::hypot(plan.target.x - world.ball.position.x,
                      plan.target.y - world.ball.position.y) <= 1.5 + 1e-9);
    assert(insideField(world, plan.target));
    assert(tactics::segmentClearance(world, world.ball.position, plan.target) >= 0.02);
}

void boundaryTargetsStayInsideField()
{
    World world = makeWorld();
    world.ball.position = {4.3, 0.0};
    const KickPlan goalPlan = tactics::chooseKick(world);
    assert(goalPlan.valid && goalPlan.action == Action::Shoot);
    assert(insideField(world, goalPlan.target));
    assert(std::abs(goalPlan.target.x - world.field.length / 2.0) < 1e-9);
    assert(std::abs(goalPlan.target.y) <= world.field.goalWidth / 2.0 - 0.2 + 1e-9);

    world.ball.position = {0.0, world.field.width / 2.0 - 0.01};
    const KickPlan carryPlan = tactics::chooseKick(world, tactics::kPi);
    assert(carryPlan.valid && carryPlan.action == Action::Carry);
    assert(insideField(world, carryPlan.target));
    assert(carryPlan.target.x > world.ball.position.x);
    assert(std::hypot(carryPlan.target.x - world.ball.position.x,
                      carryPlan.target.y - world.ball.position.y) <= 1.5 + 1e-9);
}

void disksBehindTargetDoNotOccludeShot()
{
    World world = makeWorld();
    world.obstacles[0] = {{5.2, 0.0}, 0.2, false};
    world.obstacleCount = 1;

    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Shoot);
    assert(std::abs(plan.direction) < 1e-9);
    assert(plan.clearance >= 0.02);
}

void pressureClockUsesOpponentsAndRequestsSafeClearance()
{
    World world = makeWorld();
    world.obstacles[0] = {{0.0, 1.2}, 0.24, true};
    world.obstacleCount = 1;

    const KickPlan plan = tactics::chooseKick(world);
    assert(plan.valid && plan.action == Action::Clear);
    assert(std::abs(plan.opponentArrivalSeconds - 0.8) < 1e-9);
    assert(plan.target.x > world.ball.position.x);
    assert(tactics::segmentClearance(world, world.ball.position, plan.target) >= 0.02);

    world.obstacles[0].opponent = false;
    world.obstacles[0].position = {0.2, 0.0};
    world.obstacles[0].radius = 0.1;
    const KickPlan nonOpponentPlan = tactics::chooseKick(world);
    assert(nonOpponentPlan.valid && nonOpponentPlan.action == Action::Carry);
    assert(std::isinf(nonOpponentPlan.opponentArrivalSeconds));
}

void fullyBlockedRoutesDoNotInventFallback()
{
    World world = makeWorld();
    world.obstacles[0] = {world.ball.position, 1.0, false};
    world.obstacleCount = 1;
    assertInvalidHold(tactics::chooseKick(world));
}

void invalidInputsReturnInvalidHold()
{
    World world = makeWorld();
    world.ball.known = false;
    assertInvalidHold(tactics::chooseKick(world));

    world = makeWorld();
    world.robot.theta = std::numeric_limits<double>::quiet_NaN();
    assertInvalidHold(tactics::chooseKick(world));

    world = makeWorld();
    world.obstacles[0].position.x = std::numeric_limits<double>::infinity();
    world.obstacleCount = 1;
    assertInvalidHold(tactics::chooseKick(world));

    world = makeWorld();
    world.ball.velocityKnown = true;
    world.ball.velocity.y = std::numeric_limits<double>::quiet_NaN();
    assertInvalidHold(tactics::chooseKick(world));

    world = makeWorld();
    world.ball.position.x = world.field.length / 2.0;
    assertInvalidHold(tactics::chooseKick(world));

    world = makeWorld();
    world.ball.position.x = -world.field.length / 2.0 - 0.01;
    assertInvalidHold(tactics::chooseKick(world));

    world = makeWorld();
    assertInvalidHold(tactics::chooseKick(world, std::numeric_limits<double>::quiet_NaN()));
    assertInvalidHold(tactics::chooseKick(world, -0.01));
    assertInvalidHold(tactics::chooseKick(world, tactics::kPi + 0.01));
}

} // namespace

int main()
{
    openGoalUsesBallOriginAndEvidenceGatesVisualKick();
    centerBlockerUsesOnlyClearRoutes();
    leftBlockerSelectsRightGoalLane();
    tooNarrowWindowSuppressesVisualKick();
    exactGoalWindowThresholdIsNotInflatedBySampling();
    multipleBlockersLeaveOnlySafeGroundRoutes();
    fullObstacleCapacityStillFindsSafeGroundRoute();
    invalidStaleAndDegenerateFootprintsDisableOnlyVisualKick();
    ownPenaltyDangerClearsOnlyIntoLegalForwardSpace();
    boundaryTargetsStayInsideField();
    disksBehindTargetDoNotOccludeShot();
    pressureClockUsesOpponentsAndRequestsSafeClearance();
    fullyBlockedRoutesDoNotInventFallback();
    invalidInputsReturnInvalidHold();
    return 0;
}
