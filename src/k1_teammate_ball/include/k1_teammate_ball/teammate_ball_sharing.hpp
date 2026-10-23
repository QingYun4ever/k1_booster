#pragma once

#include <map>

namespace k1_teammate_ball {

// 单条球观测报告 (来自某个队友, 或自己)。纯数据, 不依赖 ROS / brain 类型, 便于独立复用与单测。
struct BallReport {
    int player_id = 0;
    double x = 0.0;              // 球在球场坐标系的位置
    double y = 0.0;
    double confidence = 0.0;     // 0~100, 视觉置信度
    double range = 0.0;          // 该机器人到球的距离 (m)
    bool detected = false;       // 当前帧是否检测到球
    bool location_known = false; // 该机器人是否认为球位已知
    double robot_x = 0.0;        // 该机器人在球场坐标系的位置 (用于自距门控)
    double robot_y = 0.0;
    double stamp_sec = 0.0;      // 报告时间戳 (秒), 用于判定是否过期
};

// 融合后的"队伍球"结果
struct FusedBall {
    bool reliable = false;      // 是否得到可信的队友高置信度球位
    int source_player_id = 0;   // 采用的队友 playerId (0 表示无来源/沿用上次)
    double x = 0.0;
    double y = 0.0;
    double confidence = 0.0;
    bool is_fresh = false;      // 本次是否首次融合该观测
    double source_stamp_sec = 0.0; // 实际球观测时间戳 (秒), 保持期不改变
};

struct Config {
    bool enable = false;                   // 是否启用高置信度球位共享
    double min_confidence_to_share = 50.0; // 自己置信度高于此值才视为"可共享" (发送侧/调试用)
    double min_confidence_to_trust = 50.0; // 队友球置信度高于此值才纳入融合 (核心: 高置信度门控)
    double trust_timeout_sec = 1.0;        // 无新可信来源时, 维持上次结果的最长时间
    double self_dist_threshold = 2.0;      // 队友球须距我超过此值才采信
    double teammate_timeout_sec = 1.0;     // 队友报告超过此时长视为过期
    bool require_location_known = true;    // 是否要求队友 location_known 才采信
};

/**
 * @brief 队友间高置信度球位置共享 / 融合 (纯逻辑, 独立于 brain 与 ROS)。
 *
 * 设计说明:
 *  - 传输沿用 brain 既有的 UDP 通讯 (TeamCommunicationMsg 已携带 ballPosToField/ballConfidence/...),
 *    本类不持有 socket, 只负责"高置信度门控 + 选择/融合 + 超时保持"的策略;
 *  - 这样既把"是否采信、采信谁"的判定逻辑收敛到独立 package, 又复用了已验证的跨机传输;
 *  - 与旧逻辑的关键区别: 旧逻辑仅按 ballDetected + range 选最近的队友球, 不看置信度;
 *    本类增加 confidence 门控, 并优先选择置信度最高的队友球。
 */
class TeammateBallSharing {
public:
    void setConfig(const Config &cfg) { cfg_ = cfg; }
    const Config &config() const { return cfg_; }

    // 发送侧门控: 自己的球是否达到"可共享"标准 (确实看到 且 置信度足够)。
    bool shouldShare(double local_confidence, bool local_detected) const;

    // 录入一条有效队友报告; now_sec 为接收端当前时间, 同一 player_id 仅接受严格更新的观测时间戳。
    void ingest(const BallReport &report, double now_sec);

    // 基于已录入的队友报告 + 已知自身位姿, 计算融合后的队伍球位。
    // now_sec 反向跳变时会清空此前报告与保持状态。
    FusedBall fuse(double self_x, double self_y, bool self_location_known, double now_sec);

    // 清空状态 (例如重新定位 / 比赛重置时)。
    void reset();

private:
    struct StoredReport {
        BallReport report;
        bool consumed = false;
    };

    Config cfg_;
    std::map<int, StoredReport> reports_; // player_id -> 最新报告及是否已作为新观测融合
    FusedBall last_reliable_;             // 上次可信结果 (保持截止时间由 source_stamp_sec 决定)
    double last_now_sec_ = 0.0;
    bool has_last_now_sec_ = false;
};

} // namespace k1_teammate_ball
