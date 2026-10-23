#pragma once

#include <cmath>
#include <cstdint>

namespace match_policy {

enum class Primary : std::uint8_t { Initial, Ready, Set, Play, End };
enum class Phase : std::uint8_t { Normal, PenaltyShootout, ExtraTime, Timeout };
enum class Restart : std::uint8_t { None, Direct, Indirect, Penalty, ThrowIn, GoalKick, Corner };

struct Input {
    std::uint16_t version = 0;
    std::uint8_t primary = 0;
    std::uint8_t phase = 0;
    std::uint8_t setPlay = 0;
    std::uint8_t legacySecondary = 0;
    std::uint8_t legacyStage = 0;
    bool stopped = true;
    bool ownKickoff = false;
    bool ownSetPlay = false;
    bool penalized = true;
    int secondarySeconds = 0;
};

struct State {
    bool valid = false;
    bool modern = false;
    Primary primary = Primary::Initial;
    Phase phase = Phase::Normal;
    Restart restart = Restart::None;
    bool stopped = true;
    bool ownKickoff = false;
    bool ownRestart = false;
    bool penalized = true;
    std::uint8_t legacyStage = 0;
    int secondarySeconds = 0;
};

inline bool newerPacket(std::uint8_t candidate, std::uint8_t previous) {
    const auto difference = static_cast<std::uint8_t>(candidate - previous);
    return difference != 0 && difference < 128;
}

inline State interpret(const Input &input) {
    State result;
    if ((input.version != 12 && input.version != 20) || input.primary > 4) return result;
    result.modern = input.version == 20;
    result.primary = static_cast<Primary>(input.primary);
    result.ownKickoff = input.ownKickoff;
    result.ownRestart = result.modern ? input.ownKickoff : input.ownSetPlay;
    result.penalized = input.penalized;
    result.secondarySeconds = input.secondarySeconds;
    if (result.modern) {
        if (input.phase > 3 || input.setPlay > 6) return State{};
        result.phase = static_cast<Phase>(input.phase);
        result.restart = static_cast<Restart>(input.setPlay);
        result.stopped = input.stopped;
    } else {
        if (input.legacySecondary > 9 || (input.legacySecondary >= 4 && input.legacyStage > 2)) return State{};
        result.legacyStage = input.legacyStage;
        if (input.legacySecondary <= 3) {
            result.phase = static_cast<Phase>(input.legacySecondary);
        } else {
            switch (input.legacySecondary) {
            case 4: result.restart = Restart::Direct; break;
            case 5: result.restart = Restart::Indirect; break;
            case 6: result.restart = Restart::Penalty; break;
            case 7: result.restart = Restart::Corner; break;
            case 8: result.restart = Restart::GoalKick; break;
            case 9: result.restart = Restart::ThrowIn; break;
            }
        }
        result.stopped = input.stopped || (result.restart != Restart::None && input.legacyStage != 1);
    }
    if (result.phase == Phase::PenaltyShootout) result.ownRestart = input.ownKickoff;
    result.valid = true;
    return result;
}

inline bool canMove(const State &state, bool received, double ageMs, double timeoutMs,
                    bool autonomous) {
    // Offline manual/chase diagnostics remain possible; autonomous games fail closed.
    if (!received) return !autonomous;
    if (!state.valid || !std::isfinite(timeoutMs) || timeoutMs <= 0.0 ||
        !std::isfinite(ageMs) || ageMs < 0.0 || ageMs > timeoutMs ||
        state.stopped || state.penalized || state.phase == Phase::Timeout) return false;
    return state.primary == Primary::Ready || state.primary == Primary::Play;
}

inline bool canKick(const State &state, bool received, double ageMs, double timeoutMs,
                    bool autonomous) {
    if (!canMove(state, received, ageMs, timeoutMs, autonomous)) return false;
    if (!received) return true;
    if (state.primary != Primary::Play) return false;
    if (state.phase == Phase::PenaltyShootout) return state.ownKickoff;
    if (state.restart == Restart::None) {
        return !state.modern || state.ownKickoff || state.secondarySeconds <= 0;
    }
    // Legacy free-kick stages only permit positioning; PLAY begins when it clears.
    return state.modern && state.ownRestart;
}

inline const char *primaryName(Primary value) {
    switch (value) {
    case Primary::Initial: return "INITIAL";
    case Primary::Ready: return "READY";
    case Primary::Set: return "SET";
    case Primary::Play: return "PLAY";
    case Primary::End: return "END";
    }
    return "INITIAL";
}

inline const char *restartName(const State &state) {
    switch (state.restart) {
    case Restart::Direct: return "DIRECT_FREEKICK";
    case Restart::Indirect: return "INDIRECT_FREEKICK";
    case Restart::Penalty: return "PENALTY_KICK";
    case Restart::ThrowIn: return "THROW_IN";
    case Restart::GoalKick: return "GOAL_KICK";
    case Restart::Corner: return "CORNER_KICK";
    case Restart::None: break;
    }
    if (state.phase == Phase::PenaltyShootout) return "PENALTYSHOOT";
    if (state.phase == Phase::ExtraTime) return "OVERTIME";
    if (state.phase == Phase::Timeout) return "TIMEOUT";
    return "NONE";
}

} // namespace match_policy
