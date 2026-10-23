#include <cassert>
#include <cmath>

#include "tactical_decision.h"

namespace {

using tactics::Action;
using tactics::Decision;
using tactics::DecisionInput;

DecisionInput makeInput()
{
    DecisionInput input;
    input.plan.valid = true;
    input.plan.action = Action::Shoot;
    input.plan.target = {4.5, 0.0};
    input.plan.direction = 0.0;
    input.robot = {0.0, 0.0, 0.0};
    input.ball = {1.0, 0.0};
    input.range = 1.0;
    input.motionAllowed = true;
    input.ballKnown = true;
    input.localFresh = true;
    input.lead = true;
    return input;
}

void safetyPrecedenceAndBlockedPlansHold()
{
    DecisionInput input = makeInput();
    input.motionAllowed = false;
    input.ballKnown = false;
    input.lead = false;
    assert(tactics::chooseDecision(input) == Decision::Hold);

    input.motionAllowed = true;
    assert(tactics::chooseDecision(input) == Decision::Find);

    input.ballKnown = true;
    assert(tactics::chooseDecision(input) == Decision::Assist);

    input.lead = true;
    input.localFresh = false;
    input.plan.valid = false;
    assert(tactics::chooseDecision(input) == Decision::Chase);

    input.localFresh = true;
    assert(tactics::chooseDecision(input) == Decision::Hold);

    input.plan.valid = true;
    input.plan.action = Action::Hold;
    assert(tactics::chooseDecision(input) == Decision::Hold);
}

void sharedOnlyObservationChasesInsteadOfKicking()
{
    DecisionInput input = makeInput();
    input.localFresh = false;
    input.kickAllowed = true;
    input.enableVisual = true;
    input.plan.visualKickAllowed = true;
    assert(tactics::chooseDecision(input) == Decision::Chase);
}

void visualShootRequiresPermissionEvidenceAndAlignment()
{
    DecisionInput eligible = makeInput();
    eligible.kickAllowed = true;
    eligible.enableVisual = true;
    eligible.plan.visualKickAllowed = true;
    eligible.plan.direction = 0.35;
    eligible.yaw = eligible.visualYaw;
    assert(tactics::chooseDecision(eligible) == Decision::VisualKick);

    eligible.plan.direction = 0.350001;
    assert(tactics::chooseDecision(eligible) == Decision::Adjust);

    DecisionInput notKickAllowed = makeInput();
    notKickAllowed.enableVisual = true;
    notKickAllowed.plan.visualKickAllowed = true;
    assert(tactics::chooseDecision(notKickAllowed) == Decision::Adjust);

    DecisionInput disabled = makeInput();
    disabled.kickAllowed = true;
    disabled.plan.visualKickAllowed = true;
    assert(tactics::chooseDecision(disabled) == Decision::Adjust);

    DecisionInput noVisualEvidence = makeInput();
    noVisualEvidence.kickAllowed = true;
    noVisualEvidence.enableVisual = true;
    assert(tactics::chooseDecision(noVisualEvidence) == Decision::Adjust);

    DecisionInput yawOutsideVisualLimit = makeInput();
    yawOutsideVisualLimit.kickAllowed = true;
    yawOutsideVisualLimit.enableVisual = true;
    yawOutsideVisualLimit.plan.visualKickAllowed = true;
    yawOutsideVisualLimit.yaw = 0.81;
    assert(tactics::chooseDecision(yawOutsideVisualLimit) == Decision::Adjust);

    DecisionInput atVisualRangeLimit = makeInput();
    atVisualRangeLimit.kickAllowed = true;
    atVisualRangeLimit.enableVisual = true;
    atVisualRangeLimit.plan.visualKickAllowed = true;
    atVisualRangeLimit.range = atVisualRangeLimit.visualMaxRange;
    assert(tactics::chooseDecision(atVisualRangeLimit) == Decision::Chase);
}

void safeGroundKickRemainsAvailableWithoutVisualEvidence()
{
    DecisionInput input = makeInput();
    input.range = 0.5;
    input.kickAllowed = true;
    input.enableVisual = true;
    input.plan.visualKickAllowed = false;
    assert(tactics::chooseDecision(input) == Decision::Kick);

    input.plan.visualKickAllowed = true;
    input.range = input.visualMinRange;
    assert(tactics::chooseDecision(input) == Decision::Kick);
}

void groundKickRequiresApproachHeadingAndYawAlignment()
{
    DecisionInput aligned = makeInput();
    aligned.range = 0.5;
    aligned.kickAllowed = true;
    aligned.plan.direction = 0.2;
    aligned.ball = {std::cos(0.09), std::sin(0.09)};
    aligned.yaw = 0.2;
    assert(tactics::chooseDecision(aligned) == Decision::Kick);

    DecisionInput approachOutside = makeInput();
    approachOutside.range = 0.5;
    approachOutside.kickAllowed = true;
    approachOutside.plan.direction = 0.2;
    approachOutside.ball = {std::cos(0.079), std::sin(0.079)};
    assert(tactics::chooseDecision(approachOutside) == Decision::Adjust);

    DecisionInput headingOutside = makeInput();
    headingOutside.range = 0.5;
    headingOutside.kickAllowed = true;
    headingOutside.plan.direction = 0.2;
    headingOutside.robot.theta = -0.001;
    headingOutside.ball = {std::cos(0.09), std::sin(0.09)};
    assert(tactics::chooseDecision(headingOutside) == Decision::Adjust);

    DecisionInput yawOutside = makeInput();
    yawOutside.range = 0.5;
    yawOutside.kickAllowed = true;
    yawOutside.yaw = 0.201;
    assert(tactics::chooseDecision(yawOutside) == Decision::Adjust);
}

void carryClearAndKickoffGuardNeverUseVisualKick()
{
    DecisionInput carry = makeInput();
    carry.range = 0.5;
    carry.kickAllowed = true;
    carry.enableVisual = true;
    carry.plan.visualKickAllowed = true;
    carry.plan.action = Action::Carry;
    assert(tactics::chooseDecision(carry) == Decision::Cross);

    DecisionInput clear = makeInput();
    clear.range = 0.5;
    clear.kickAllowed = true;
    clear.enableVisual = true;
    clear.plan.visualKickAllowed = true;
    clear.plan.action = Action::Clear;
    assert(tactics::chooseDecision(clear) == Decision::Cross);

    DecisionInput guard = makeInput();
    guard.range = 0.5;
    guard.kickAllowed = true;
    guard.enableVisual = true;
    guard.plan.visualKickAllowed = true;
    guard.kickoffGuard = true;
    assert(tactics::chooseDecision(guard) == Decision::Cross);
}

void chaseHysteresisUsesReducedThresholdOnlyWhenAlreadyChasing()
{
    DecisionInput approaching = makeInput();
    approaching.range = 0.95;
    approaching.kickRange = 0.5;
    assert(tactics::chooseDecision(approaching) == Decision::Adjust);

    DecisionInput continuing = makeInput();
    continuing.range = 0.95;
    continuing.kickRange = 0.5;
    continuing.wasChasing = true;
    assert(tactics::chooseDecision(continuing) == Decision::Chase);

    continuing.range = 0.9;
    assert(tactics::chooseDecision(continuing) == Decision::Adjust);
}

void pressureShortensAdjustmentTimeout()
{
    DecisionInput normal = makeInput();
    normal.range = 0.8;
    normal.kickRange = 0.5;
    normal.configuredAdjustTimeout = 1.0;
    normal.adjustSeconds = 0.8;
    assert(tactics::chooseDecision(normal) == Decision::Adjust);

    DecisionInput pressured = makeInput();
    pressured.range = 0.8;
    pressured.kickRange = 0.5;
    pressured.configuredAdjustTimeout = 1.0;
    pressured.adjustSeconds = 0.8;
    pressured.plan.opponentArrivalSeconds = 0.8;
    assert(tactics::chooseDecision(pressured) == Decision::Chase);
}

} // namespace

int main()
{
    safetyPrecedenceAndBlockedPlansHold();
    sharedOnlyObservationChasesInsteadOfKicking();
    visualShootRequiresPermissionEvidenceAndAlignment();
    safeGroundKickRemainsAvailableWithoutVisualEvidence();
    groundKickRequiresApproachHeadingAndYawAlignment();
    carryClearAndKickoffGuardNeverUseVisualKick();
    chaseHysteresisUsesReducedThresholdOnlyWhenAlreadyChasing();
    pressureShortensAdjustmentTimeout();
    return 0;
}
