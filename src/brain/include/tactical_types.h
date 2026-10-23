#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace tactics {
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxObstacles = 32;

struct Point { double x = 0.0, y = 0.0; };
struct Pose { double x = 0.0, y = 0.0, theta = 0.0; };
struct Field {
    double length = 0.0, width = 0.0, goalWidth = 0.0;
    double penaltyLength = 0.0, penaltyWidth = 0.0;
};
struct Limits { double forward = 0.9, lateral = 0.2, turn = 1.0; };
struct Ball {
    Point position;
    Point velocity;
    bool velocityKnown = false;
    double ageSeconds = 0.0;
    double confidence = 0.0;
    bool known = false;
};
struct Obstacle {
    Point position;
    double radius = 0.45; // Robot footprint plus observation uncertainty, meters.
    bool opponent = false;
};
struct World {
    Field field;
    Limits limits;
    Pose robot;
    Ball ball;
    Pose owner;
    bool ownerKnown = false;
    bool ownerControlling = false;
    bool goalObserved = false;
    bool viewReliable = false;
    std::array<Point, 4> viewFootprint{}; // Fresh camera ground quadrilateral, field coordinates.
    std::array<Obstacle, kMaxObstacles> obstacles{};
    std::size_t obstacleCount = 0;
};

enum class Action { Hold, Carry, Shoot, Clear };
struct KickPlan {
    bool valid = false;
    Action action = Action::Hold;
    Point target;
    double direction = 0.0;
    double clearance = 0.0;
    double windowRadians = 0.0;
    double opponentArrivalSeconds = std::numeric_limits<double>::infinity();
    bool visualKickAllowed = false;
};
enum class SupportRole { Cover, Lateral, Rebound };
struct SupportPlan {
    bool valid = false;
    Pose target;
    SupportRole role = SupportRole::Cover;
    Point referenceBall;
    Point referenceOwner;
};

inline bool finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y); }
inline bool finite(Pose p) { return finite(Point{p.x, p.y}) && std::isfinite(p.theta); }
inline double distance(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }
inline Point point(Pose p) { return {p.x, p.y}; }
inline double angle(Point from, Point to) { return std::atan2(to.y - from.y, to.x - from.x); }
inline double wrap(double a) { return std::remainder(a, 2.0 * kPi); }
inline bool validField(const Field &f) {
    return std::isfinite(f.length) && std::isfinite(f.width) && std::isfinite(f.goalWidth) &&
        std::isfinite(f.penaltyLength) && std::isfinite(f.penaltyWidth) &&
        f.length > 2.0 && f.width > 2.0 && f.goalWidth > 0.5 && f.goalWidth < f.width &&
        f.penaltyLength > 0.0 && f.penaltyLength < f.length / 2.0 &&
        f.penaltyWidth > 0.0 && f.penaltyWidth < f.width;
}
inline bool validBall(const Ball &b) {
    return b.known && finite(b.position) && std::isfinite(b.ageSeconds) &&
        b.ageSeconds >= 0.0 && b.ageSeconds <= 3.0 && std::isfinite(b.confidence) &&
        b.confidence >= 0.0 && b.confidence <= 100.0;
}
inline bool validLimits(Limits l) {
    return std::isfinite(l.forward) && std::isfinite(l.lateral) && std::isfinite(l.turn) &&
        l.forward > 0.0 && l.lateral > 0.0 && l.turn > 0.0;
}
inline double pointSegmentDistance(Point p, Point a, Point b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 <= 1e-12) return distance(p, a);
    const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.0, 1.0);
    return distance(p, {a.x + t * dx, a.y + t * dy});
}
inline double segmentClearance(const World &world, Point from, Point target) {
    double result = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < std::min(world.obstacleCount, kMaxObstacles); ++i) {
        const auto &o = world.obstacles[i];
        if (!finite(o.position) || !std::isfinite(o.radius) || o.radius < 0.0) return -1.0;
        result = std::min(result, pointSegmentDistance(o.position, from, target) - o.radius);
    }
    return result;
}
inline const char *actionName(Action a) {
    switch (a) {
    case Action::Hold: return "hold";
    case Action::Carry: return "carry";
    case Action::Shoot: return "shoot";
    case Action::Clear: return "clear";
    }
    return "hold";
}
inline const char *supportName(SupportRole r) {
    switch (r) {
    case SupportRole::Cover: return "cover";
    case SupportRole::Lateral: return "lateral";
    case SupportRole::Rebound: return "rebound";
    }
    return "cover";
}
} // namespace tactics
