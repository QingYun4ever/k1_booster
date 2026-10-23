#include <cassert>
#include <cmath>
#include <limits>

#include "tactical_support.h"

namespace {

using tactics::Ball;
using tactics::Field;
using tactics::Obstacle;
using tactics::Point;
using tactics::Pose;
using tactics::SupportPlan;
using tactics::SupportRole;
using tactics::World;

World makeWorld()
{
    World world;
    world.field = Field{9.0, 6.0, 1.5, 1.0, 2.0};
    world.robot = Pose{-2.5, -2.0, 0.0};
    world.ball = Ball{Point{0.0, 0.0}, Point{}, false, 0.1, 90.0, true};
    world.owner = Pose{-0.6, 0.0, 0.0};
    world.ownerKnown = true;
    world.obstacles[0] = Obstacle{Point{world.owner.x, world.owner.y}, 0.45, false};
    world.obstacleCount = 1;
    return world;
}

void setOwner(World &world, Pose owner)
{
    world.owner = owner;
    if (world.obstacleCount > 0) world.obstacles[0].position = Point{owner.x, owner.y};
}

bool inPenalty(Point target, const Field &field, bool attacking)
{
    const double boundary = attacking ? field.length / 2.0 - field.penaltyLength
        : -field.length / 2.0 + field.penaltyLength;
    return (attacking ? target.x >= boundary : target.x <= boundary) &&
        std::abs(target.y) <= field.penaltyWidth / 2.0;
}

void assertLegal(const World &world, const SupportPlan &plan)
{
    assert(plan.valid);
    assert(tactics::finite(plan.target));
    const Point target{plan.target.x, plan.target.y};
    const double halfLength = world.field.length / 2.0;
    const double halfWidth = world.field.width / 2.0;
    assert(target.x >= -halfLength + 0.6 && target.x <= halfLength - 0.6);
    assert(target.y >= -halfWidth + 0.6 && target.y <= halfWidth - 0.6);
    assert(!inPenalty(target, world.field, false));
    assert(!inPenalty(target, world.field, true));
    assert(tactics::distance(target, world.ball.position) >= 1.2);
    assert(tactics::distance(target, Point{world.owner.x, world.owner.y}) >= 1.0);
    assert(tactics::pointSegmentDistance(target, world.ball.position,
        Point{world.field.length / 2.0, 0.0}) >= 0.9);
    assert(tactics::support_detail::obstaclePathClear(world,
        Point{world.robot.x, world.robot.y}, target));
    assert(std::isfinite(plan.target.theta));
    assert(std::abs(tactics::wrap(plan.target.theta -
        tactics::angle(target, world.ball.position))) < 1e-9);
}

void testOwnHalfCoversGoalChannelAndKeepsShotLaneClear()
{
    const World world = makeWorld();
    const SupportPlan plan = tactics::chooseSupport(world);
    assertLegal(world, plan);
    assert(plan.target.x < world.ball.position.x);
    assert(tactics::pointSegmentDistance(Point{plan.target.x, plan.target.y},
        world.ball.position, Point{-world.field.length / 2.0, 0.0}) < 0.5);
    assert(tactics::pointSegmentDistance(Point{plan.target.x, plan.target.y},
        world.ball.position, Point{world.field.length / 2.0, 0.0}) >= 0.9);
}

void testMirroringAndAttackingRebound()
{
    World left = makeWorld();
    left.ball.position.y = 0.25;
    left.robot.y = 2.0;
    setOwner(left, Pose{-0.6, 0.1, 0.0});
    World right = left;
    right.ball.position.y = -left.ball.position.y;
    right.robot.y = -left.robot.y;
    setOwner(right, Pose{left.owner.x, -left.owner.y, 0.0});

    const SupportPlan leftPlan = tactics::chooseSupport(left);
    const SupportPlan rightPlan = tactics::chooseSupport(right);
    assertLegal(left, leftPlan);
    assertLegal(right, rightPlan);
    assert(std::abs(leftPlan.target.x - rightPlan.target.x) < 1e-9);
    assert(std::abs(leftPlan.target.y + rightPlan.target.y) < 1e-9);
    assert(leftPlan.role == rightPlan.role);

    World attack = makeWorld();
    attack.ball.position = Point{2.0, 0.0};
    attack.robot = Pose{2.8, 1.8, 0.0};
    setOwner(attack, Pose{1.4, 0.0, 0.0});
    attack.ownerControlling = true;
    const SupportPlan rebound = tactics::chooseSupport(attack);
    assertLegal(attack, rebound);
    assert(rebound.role == SupportRole::Rebound);
    assert(rebound.target.x > attack.ball.position.x);

    attack.ownerControlling = false;
    const SupportPlan support = tactics::chooseSupport(attack);
    assertLegal(attack, support);
    assert(support.role != SupportRole::Rebound);
    assert(support.target.x < attack.ball.position.x);
}

void testEdgeAndGoalPositionsRemainLegal()
{
    World ownGoal = makeWorld();
    ownGoal.ball.position = Point{-ownGoal.field.length / 2.0, 0.0};
    ownGoal.robot = Pose{-2.5, 2.0, 0.0};
    setOwner(ownGoal, Pose{-3.2, 0.0, 0.0});
    const SupportPlan ownPlan = tactics::chooseSupport(ownGoal);
    assertLegal(ownGoal, ownPlan);
    assert(std::abs(ownPlan.target.y - ownGoal.ball.position.y) >= 1.2);

    World attackingGoal = makeWorld();
    attackingGoal.ball.position = Point{attackingGoal.field.length / 2.0 - 0.3, 0.0};
    attackingGoal.robot = Pose{2.8, -2.0, 0.0};
    setOwner(attackingGoal, Pose{3.1, 0.0, 0.0});
    const SupportPlan attackingPlan = tactics::chooseSupport(attackingGoal);
    assertLegal(attackingGoal, attackingPlan);

    World sideline = makeWorld();
    sideline.ball.position = Point{0.0, 2.8};
    sideline.robot = Pose{-2.5, 2.0, 0.0};
    setOwner(sideline, Pose{-0.6, 2.2, 0.0});
    const SupportPlan sidelinePlan = tactics::chooseSupport(sideline);
    assertLegal(sideline, sidelinePlan);
}

void testOwnerApproachAndObstacleAvoidance()
{
    World world = makeWorld();
    world.owner = Pose{-1.0, 0.0, 0.0};
    world.ball.position = Point{0.5, 0.0};
    setOwner(world, world.owner);
    const SupportPlan approachPlan = tactics::chooseSupport(world);
    assertLegal(world, approachPlan);
    assert(tactics::pointSegmentDistance(Point{approachPlan.target.x, approachPlan.target.y},
        Point{world.owner.x, world.owner.y}, world.ball.position) >= 0.9);

    World obstacleWorld = makeWorld();
    const SupportPlan initial = tactics::chooseSupport(obstacleWorld);
    assertLegal(obstacleWorld, initial);
    obstacleWorld.obstacles[obstacleWorld.obstacleCount++] = Obstacle{
        Point{initial.target.x, initial.target.y}, 0.7, true};
    const SupportPlan detour = tactics::chooseSupport(obstacleWorld);
    assertLegal(obstacleWorld, detour);
    assert(tactics::distance(Point{detour.target.x, detour.target.y},
        obstacleWorld.obstacles[1].position) > obstacleWorld.obstacles[1].radius);
    assert(tactics::distance(Point{detour.target.x, detour.target.y},
        Point{initial.target.x, initial.target.y}) > 0.7);
}


void testCanLeaveOwnerCorridorByIncreasingSeparation()
{
    World world = makeWorld();
    setOwner(world, Pose{-1.0, 0.0, 0.0});
    world.ball.position = Point{0.5, 0.0};
    world.robot = Pose{-0.85, 0.15, 0.0};

    const SupportPlan plan = tactics::chooseSupport(world);
    assertLegal(world, plan);
    const Point from{world.robot.x, world.robot.y};
    const Point target{plan.target.x, plan.target.y};
    const Point owner{world.owner.x, world.owner.y};
    const Point fromOffset{from.x - owner.x, from.y - owner.y};
    const Point travel{target.x - from.x, target.y - from.y};
    assert(fromOffset.x * travel.x + fromOffset.y * travel.y >= 0.0);
    assert(tactics::distance(target, owner) > tactics::distance(from, owner));
}

void testPreviousTargetStabilityAndBallMotionRefresh()
{
    World world = makeWorld();
    const SupportPlan initial = tactics::chooseSupport(world);
    assertLegal(world, initial);

    world.ball.position.x += 0.15;
    const SupportPlan held = tactics::chooseSupport(world, &initial);
    assertLegal(world, held);
    assert(std::abs(held.target.x - initial.target.x) < 1e-9);
    assert(std::abs(held.target.y - initial.target.y) < 1e-9);
    assert(held.referenceBall.x == initial.referenceBall.x);
    World escaped = world;
    escaped.ball.position.x += 0.3;
    const SupportPlan refreshed = tactics::chooseSupport(escaped, &held);
    assertLegal(escaped, refreshed);
    assert(refreshed.referenceBall.x == escaped.ball.position.x);

    SupportPlan invalidPrevious = initial;
    invalidPrevious.target.x = std::numeric_limits<double>::quiet_NaN();
    const SupportPlan replanned = tactics::chooseSupport(world, &invalidPrevious);
    const SupportPlan fresh = tactics::chooseSupport(world);
    assertLegal(world, replanned);
    assert(std::abs(replanned.target.x - fresh.target.x) < 1e-9);
    assert(std::abs(replanned.target.y - fresh.target.y) < 1e-9);

    world.ball.position = Point{2.4, 1.0};
    setOwner(world, Pose{1.7, 0.5, 0.0});
    const SupportPlan moved = tactics::chooseSupport(world, &initial);
    assertLegal(world, moved);
    assert(tactics::distance(Point{moved.target.x, moved.target.y},
        Point{initial.target.x, initial.target.y}) > 1.1);
}

void testInvalidWorldsReturnFiniteHold()
{
    World world = makeWorld();
    world.ownerKnown = false;
    const SupportPlan noOwner = tactics::chooseSupport(world);
    assert(!noOwner.valid);
    assert(tactics::finite(noOwner.target));

    world = makeWorld();
    world.ball.known = false;
    const SupportPlan noBall = tactics::chooseSupport(world);
    assert(!noBall.valid);
    assert(tactics::finite(noBall.target));

    world = makeWorld();
    world.robot.x = std::numeric_limits<double>::quiet_NaN();
    const SupportPlan invalidRobot = tactics::chooseSupport(world);
    assert(!invalidRobot.valid);
    assert(tactics::finite(invalidRobot.target));

    world = makeWorld();
    world.owner.y = std::numeric_limits<double>::quiet_NaN();
    const SupportPlan invalidOwner = tactics::chooseSupport(world);
    assert(!invalidOwner.valid);
    assert(tactics::finite(invalidOwner.target));
    world = makeWorld();
    world.ball.position.x = world.field.length;
    assert(!tactics::chooseSupport(world).valid);
    world = makeWorld();
    world.obstacleCount = tactics::kMaxObstacles + 1;
    assert(!tactics::chooseSupport(world).valid);
}

} // namespace

int main()
{
    testOwnHalfCoversGoalChannelAndKeepsShotLaneClear();
    testMirroringAndAttackingRebound();
    testEdgeAndGoalPositionsRemainLegal();
    testOwnerApproachAndObstacleAvoidance();
    testCanLeaveOwnerCorridorByIncreasingSeparation();
    testPreviousTargetStabilityAndBallMotionRefresh();
    testInvalidWorldsReturnFiniteHold();
    return 0;
}
