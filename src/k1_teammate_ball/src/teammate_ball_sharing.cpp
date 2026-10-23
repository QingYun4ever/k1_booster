#include "k1_teammate_ball/teammate_ball_sharing.hpp"

#include <cmath>

namespace k1_teammate_ball {
namespace {

bool isFiniteReport(const BallReport &report) {
    return report.player_id > 0
        && std::isfinite(report.x)
        && std::isfinite(report.y)
        && std::isfinite(report.confidence)
        && report.confidence >= 0.0
        && report.confidence <= 100.0
        && std::isfinite(report.range)
        && report.range >= 0.0
        && std::isfinite(report.robot_x)
        && std::isfinite(report.robot_y)
        && std::isfinite(report.stamp_sec);
}

} // namespace

bool TeammateBallSharing::shouldShare(double local_confidence, bool local_detected) const {
    return cfg_.enable
        && local_detected
        && std::isfinite(local_confidence)
        && local_confidence >= 0.0
        && local_confidence <= 100.0
        && std::isfinite(cfg_.min_confidence_to_share)
        && local_confidence >= cfg_.min_confidence_to_share;
}

void TeammateBallSharing::ingest(const BallReport &report, double now_sec) {
    if (!std::isfinite(now_sec)) return;

    if (has_last_now_sec_ && now_sec < last_now_sec_) {
        reports_.clear();
        last_reliable_ = FusedBall{};
    }
    last_now_sec_ = now_sec;
    has_last_now_sec_ = true;

    if (!isFiniteReport(report) || report.stamp_sec > now_sec) return;

    const auto it = reports_.find(report.player_id);
    if (it != reports_.end()) {
        if (report.stamp_sec <= it->second.report.stamp_sec) return;
        it->second.report = report;
        it->second.consumed = false;
        return;
    }

    reports_.emplace(report.player_id, StoredReport{report, false});
}

void TeammateBallSharing::reset() {
    reports_.clear();
    last_reliable_ = FusedBall{};
    last_now_sec_ = 0.0;
    has_last_now_sec_ = false;
}

FusedBall TeammateBallSharing::fuse(double self_x, double self_y, bool self_location_known, double now_sec) {
    FusedBall result;
    if (!std::isfinite(now_sec)) return result;

    if (has_last_now_sec_ && now_sec < last_now_sec_) {
        reports_.clear();
        last_reliable_ = FusedBall{};
    }
    last_now_sec_ = now_sec;
    has_last_now_sec_ = true;

    for (auto it = reports_.begin(); it != reports_.end();) {
        if (it->second.report.stamp_sec > now_sec) {
            it = reports_.erase(it);
        } else {
            ++it;
        }
    }

    if (!cfg_.enable || !self_location_known || !std::isfinite(self_x) || !std::isfinite(self_y)) {
        return result;
    }
    if (!std::isfinite(cfg_.min_confidence_to_trust)
        || cfg_.min_confidence_to_trust < 0.0
        || cfg_.min_confidence_to_trust > 100.0
        || !std::isfinite(cfg_.teammate_timeout_sec)
        || cfg_.teammate_timeout_sec < 0.0
        || !std::isfinite(cfg_.trust_timeout_sec)
        || cfg_.trust_timeout_sec < 0.0
        || !std::isfinite(cfg_.self_dist_threshold)
        || cfg_.self_dist_threshold < 0.0) {
        return result;
    }

    StoredReport *best = nullptr;
    for (auto &entry : reports_) {
        StoredReport &stored = entry.second;
        const BallReport &report = stored.report;
        const double age = now_sec - report.stamp_sec;

        if (!std::isfinite(age)
            || age > cfg_.teammate_timeout_sec
            || age > cfg_.trust_timeout_sec) continue;
        if (!report.detected) continue;
        if (cfg_.require_location_known && !report.location_known) continue;
        if (report.confidence < cfg_.min_confidence_to_trust) continue;

        const double self_distance = std::hypot(report.x - self_x, report.y - self_y);
        if (!std::isfinite(self_distance) || self_distance <= cfg_.self_dist_threshold) continue;

        const BallReport *best_report = best == nullptr ? nullptr : &best->report;
        if (best_report == nullptr
            || report.confidence > best_report->confidence
            || (report.confidence == best_report->confidence && report.range < best_report->range)
            || (report.confidence == best_report->confidence && report.range == best_report->range
                && report.player_id < best_report->player_id)) {
            best = &stored;
        }
    }

    if (best != nullptr) {
        const BallReport &report = best->report;
        result.reliable = true;
        result.is_fresh = !best->consumed;
        result.source_player_id = report.player_id;
        result.x = report.x;
        result.y = report.y;
        result.confidence = report.confidence;
        result.source_stamp_sec = report.stamp_sec;
        best->consumed = true;
        if (result.is_fresh) last_reliable_ = result;
        return result;
    }

    const double hold_age = now_sec - last_reliable_.source_stamp_sec;
    const double hold_distance = std::hypot(last_reliable_.x - self_x, last_reliable_.y - self_y);
    if (last_reliable_.reliable
        && std::isfinite(cfg_.trust_timeout_sec)
        && cfg_.trust_timeout_sec >= 0.0
        && std::isfinite(hold_age)
        && hold_age >= 0.0
        && hold_age <= cfg_.trust_timeout_sec && std::isfinite(hold_distance)
        && hold_distance > cfg_.self_dist_threshold) {
        result = last_reliable_;
        result.is_fresh = false;
    }
    return result;
}

} // namespace k1_teammate_ball
