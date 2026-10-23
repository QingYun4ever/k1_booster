#pragma once

#include "tactical_types.h"

namespace tactics {
enum class Decision { Hold, Find, Assist, Chase, Adjust, Kick, Cross, VisualKick };
struct DecisionInput {
    KickPlan plan;
    Pose robot;
    Point ball;
    double range = 0.0;
    double yaw = 0.0;
    double chaseThreshold = 1.0;
    double kickRange = 1.0;
    double kickYaw = 0.2;
    double visualMinRange = 0.2;
    double visualMaxRange = 4.0;
    double visualYaw = 0.8;
    double adjustSeconds = 0.0;
    double configuredAdjustTimeout = 0.0;
    bool motionAllowed = false;
    bool kickAllowed = false;
    bool ballKnown = false;
    bool localFresh = false;
    bool lead = false;
    bool kickoffGuard = false;
    bool enableVisual = false;
    bool wasChasing = false;
};
inline const char *decisionName(Decision d) {
    switch (d) {
    case Decision::Hold: return "hold";
    case Decision::Find: return "find";
    case Decision::Assist: return "assist";
    case Decision::Chase: return "chase";
    case Decision::Adjust: return "adjust";
    case Decision::Kick: return "kick";
    case Decision::Cross: return "cross";
    case Decision::VisualKick: return "auto_visual_kick";
    }
    return "hold";
}

inline Decision chooseDecision(const DecisionInput &input) {
    if (!input.motionAllowed) return Decision::Hold;
    if (!input.ballKnown) return Decision::Find;
    if (!input.lead) return Decision::Assist;
    if (!finite(input.robot) || !finite(input.ball) || !std::isfinite(input.range) || input.range < 0.0 ||
        !std::isfinite(input.yaw) || !std::isfinite(input.chaseThreshold) || input.chaseThreshold <= 0.0 ||
        !std::isfinite(input.kickRange) || input.kickRange <= 0.0 || !std::isfinite(input.kickYaw) || input.kickYaw < 0.0)
        return Decision::Hold;
    if (!input.localFresh) return Decision::Chase;
    if (!input.plan.valid || input.plan.action == Action::Hold || !finite(input.plan.target) ||
        !std::isfinite(input.plan.direction)) return Decision::Hold;
    const double approachError = std::abs(wrap(input.plan.direction - angle(point(input.robot), input.ball)));
    const double headingError = std::abs(wrap(input.plan.direction - input.robot.theta));
    if (input.kickAllowed && input.enableVisual && !input.kickoffGuard &&
        input.plan.action == Action::Shoot && input.plan.visualKickAllowed &&
        std::isfinite(input.visualMinRange) && std::isfinite(input.visualMaxRange) &&
        std::isfinite(input.visualYaw) && input.visualMinRange >= 0.0 && input.visualMaxRange > input.visualMinRange &&
        input.visualYaw >= 0.0 && input.range > input.visualMinRange && input.range < input.visualMaxRange &&
        std::abs(input.yaw) <= input.visualYaw && headingError <= 0.35 && approachError <= 0.35)
        return Decision::VisualKick;
    if (input.range > input.chaseThreshold * (input.wasChasing ? 0.9 : 1.0)) return Decision::Chase;
    if (input.kickAllowed && input.range < input.kickRange && std::abs(input.yaw) <= input.kickYaw &&
        headingError <= input.kickYaw && approachError <= 0.12) {
        return input.kickoffGuard || input.plan.action == Action::Carry || input.plan.action == Action::Clear
            ? Decision::Cross : Decision::Kick;
    }
    // Pressure shortens positioning, but never authorizes kicking into a blocked lane.
    double budget = input.configuredAdjustTimeout > 0.0 ? input.configuredAdjustTimeout
                                                       : std::numeric_limits<double>::infinity();
    if (std::isfinite(input.plan.opponentArrivalSeconds))
        budget = std::min(budget, std::max(0.35, input.plan.opponentArrivalSeconds - 0.35));
    if (std::isfinite(input.adjustSeconds) && input.adjustSeconds >= budget) return Decision::Chase;
    return Decision::Adjust;
}
} // namespace tactics
