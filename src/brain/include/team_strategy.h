#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace team_strategy {

// Fixed capacity is storage, never the number of participating players.
constexpr std::size_t kCapacity = 11;
struct Candidate {
    int id = 0;
    bool available = false;
    bool visualKick = false;
    bool claimsOwner = false;
    double cost = std::numeric_limits<double>::infinity();
};

inline int goalkeeper(const std::array<bool, kCapacity> &eligible, int count,
                      int preferred, bool takeover) {
    if (count < 1 || count > static_cast<int>(kCapacity)) return 0;
    if (preferred >= 1 && preferred <= count && eligible[preferred - 1]) return preferred;
    if (!takeover) return 0;
    for (int i = 0; i < count; ++i) if (eligible[i]) return i + 1;
    return 0;
}

inline int fallbackOwner(const std::array<bool, kCapacity> &eligible, int count, int goalie) {
    if (count < 1 || count > static_cast<int>(kCapacity)) return 0;
    for (int i = 0; i < count; ++i) if (eligible[i] && i + 1 != goalie) return i + 1;
    return 0;
}

inline bool better(const Candidate &lhs, const Candidate &rhs) {
    // Quantized cost defines a total order; pairwise epsilon comparisons do not.
    const double left = std::floor(lhs.cost / 0.05);
    const double right = std::floor(rhs.cost / 0.05);
    return left < right || (left == right && lhs.id < rhs.id);
}

inline int owner(const std::array<Candidate, kCapacity> &players, int count,
                 double switchMargin) {
    if (count < 1 || count > static_cast<int>(kCapacity) || !std::isfinite(switchMargin) || switchMargin < 0.0) return 0;
    const Candidate *best = nullptr;
    const Candidate *active = nullptr;
    const Candidate *incumbent = nullptr;
    int ownerClaims = 0;
    for (int i = 0; i < count; ++i) {
        const auto &candidate = players[i];
        if (!candidate.available || candidate.id < 1 || candidate.id > count ||
            !std::isfinite(candidate.cost) || candidate.cost < 0.0) continue;
        if (!best || better(candidate, *best)) best = &candidate;
        if (candidate.visualKick && (!active || candidate.id < active->id)) active = &candidate;
        if (candidate.claimsOwner) {
            incumbent = &candidate;
            ++ownerClaims;
        }
    }
    // Skill ownership is retained, with deterministic resolution of conflicting claims.
    if (active) return active->id;
    if (!best) return 0;
    if (ownerClaims == 1 && incumbent->cost <= best->cost + switchMargin) return incumbent->id;
    return best->id;
}

} // namespace team_strategy
