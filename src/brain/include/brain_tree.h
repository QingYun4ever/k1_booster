#pragma once

#include <tuple>
#include <vector>
#include <string>
#include <stdexcept>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>
#include <rclcpp/rclcpp.hpp> 

#include "types.h"
#include "formation.h"
#include "motion_policy.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();
    void halt();
    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value{};
        auto res = tree.rootBlackboard()->get<T>(key, value);
        if (!res) throw std::runtime_error("Missing behavior-tree blackboard entry: " + key);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;
    void initEntry();
};

// ------------------------------- 比赛用 -------------------------------


// 计算踢球的角度
class CalcKickDir : public SyncActionNode 
{
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("cross_threshold", 0.2, "可进门的角度范围") }; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            InputPort<string>("position", "offense", "offense | defense"),
            OutputPort<string>("decision_out")};
    }
    NodeStatus tick() override;
private:
    Brain *brain;
    double lastDeltaDir;
    rclcpp::Time timeLastTick;
    rclcpp::Time _adjustSince;      // adjust 连续开始的时刻 (防卡死计时)
    bool _adjustActive = false;     // 当前是否处于连续 adjust 计时中
};


class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("chase_threshold", 1.0, ""),
            InputPort<double>("adjust_angle_tolerance", 0.1, ""),
            InputPort<double>("adjust_y_tolerance", 0.1, ""), 
            InputPort<string>("decision_in", "", ""),
            OutputPort<string>("decision_out"),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
};

class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class CamFindBall : public SyncActionNode
{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);
    NodeStatus tick() override;
private:
    double _cmdSequence[6][2];
    rclcpp::Time _timeLastCmd;
    int _cmdIndex;
    long _cmdIntervalMSec;
    long _cmdRestartIntervalMSec;
    Brain *brain;
};

class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("low_pitch", 0.35, "向下看时的最大 pitch"),
            InputPort<double>("high_pitch", 0.15, "向上看时的最小 pitch"),
            InputPort<double>("left_yaw", 0.8, "向左看时的最大 yaw"),
            InputPort<double>("right_yaw", -0.8, "向右看时的最小 yaw"),
            InputPort<int>("msec_cycle", 4000, "多少毫秒转一圈"),
        };
    }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class CamFastScan : public StatefulActionNode
{
public:
    CamFastScan(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("msecs_interval", 300, "") }; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};
private:
    double _cmdSequence[7][2] = {
        {0.2, 1.1}, {0.2, 0.0}, {0.2, -1.1},
        {0.9, -1.1}, {0.9, 0.0}, {0.9, 1.1}, {0.2, 0.0},
    };
    rclcpp::Time _timeLastCmd;
    int _cmdIndex = 0;
    Brain *brain;
};

/**
 * @brief 椭圆(李萨如)平滑扫球 —— 从 1.5 迁移并修复"头部周期与转身重合"的问题。
 *
 * 与旧版 1:1 椭圆(yaw=sin, pitch=cos, 头部 pitch 与 yaw 锁死)不同:
 *   - pitch 为"快轴": 每 pitch_cycle_msec 完整上下扫一遍, 覆盖远/近, 与身体转身解耦;
 *   - yaw 为"慢轴": 周期 = pitch_cycle_msec * frequency_ratio, 与快轴非 1:1, 填满 yaw×pitch;
 *   - 检测到球立即切换为视线跟踪, 把球收到画面中心。
 * 配合 RobotFindBall 匀速转身, 只要 pitch_cycle_msec < 相机水平视场角/转身角速度,
 * 即可保证身体转一圈中每个方位的远近都被扫到。
 *
 * 注意: 本节点只用于"主动找球"(FindBall), 不替代扫场定位用的 CamScanField。
 */
class CamLissajousScan : public StatefulActionNode
{
public:
    CamLissajousScan(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("pitch_center", 0.55, "竖直扫描中心 pitch(rad), 向下为正"),
            InputPort<double>("pitch_amplitude", 0.35, "竖直扫描幅度(rad), 实际 pitch ∈ [center-amp, center+amp]"),
            InputPort<double>("yaw_amplitude", 0.9, "水平扫描幅度(rad)"),
            InputPort<double>("pitch_cycle_msec", 1000.0, "快轴(pitch)完整上下扫一遍的周期(ms), 越小竖直覆盖越密、可越快转身"),
            InputPort<double>("frequency_ratio", 3.0, "快轴(pitch):慢轴(yaw) 频率比, 须 >1 以填满 yaw×pitch、解除与转身周期的锁定"),
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};
private:
    rclcpp::Time _startTime;
    double _pitchPhase = 0.0; // onStart 时按当前头部姿态对齐相位, 避免起始跳变
    double _yawPhase = 0.0;
    Brain *brain;
};

class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("vyaw_limit", 1.0, "找球时身体旋转速度上限(rad/s)"),
            InputPort<double>("transition_vx", 0.25, "看到球但位置尚未稳定时的前进衔接速度(m/s)"),
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    double _turnDir;
    int _stableDetectedCount = 0;
    Brain *brain;
};

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("rad", 0, ""),
            InputPort<bool>("towards_ball", false, "")
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};
private:
    double _lastAngle;
    double _angle;
    double _cumAngle;
    double _msecLimit = 5000;
    rclcpp::Time _timeStart;
    Brain *brain;
};

class DecideCheckBehind : public SyncActionNode
{
public:
    DecideCheckBehind(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("vx_limit", 0.4, ""),
            InputPort<double>("vy_limit", 0.4, ""),
            InputPort<double>("vtheta_limit", 0.1, ""),
            InputPort<double>("dist", 1.0, ""),
            InputPort<double>("safe_dist", 1.0, ""),
            InputPort<bool>("arc_walk", false, "true=走弧线(sqrt(cos)朝向因子,中小偏角仍前进,边走边转); false=原行为(sigmoid,偏角大时近乎原地转)"),
        };
    }
    NodeStatus tick() override;
private:
    Brain *brain;
    string _state;
    double _dir = 1.0;
};

class Chase2 : public StatefulActionNode { public: Chase2(const string &n, const NodeConfig &c, Brain *b) : StatefulActionNode(n,c), brain(b) {} NodeStatus onStart() override { return NodeStatus::SUCCESS; } NodeStatus onRunning() override { return NodeStatus::SUCCESS; } void onHalted() override {} static PortsList providedPorts() { return {}; } private: Brain *brain; };

class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("turn_threshold", 0.2, ""),
            InputPort<double>("vx_limit", 0.1, ""),
            InputPort<double>("vy_limit", 0.1, ""),
            InputPort<double>("vtheta_limit", 0.4, ""),
            InputPort<double>("range", 1.5, ""),
            InputPort<double>("vtheta_factor", 1.5, ""),
            InputPort<double>("tangential_speed_far", 0.7, ""),
            InputPort<double>("tangential_speed_near", 0.15, ""),
            InputPort<double>("near_threshold", 0.8, ""),
            InputPort<double>("no_turn_threshold", 0.1, ""),
            InputPort<double>("turn_first_threshold", 0.5, ""),
            InputPort<double>("min_range", 1.0, ""),
            InputPort<string>("position", "offense", "")};
    }
    NodeStatus tick() override;
private:
    Brain *brain;
};

// 补充 Adjust 变体定义
class Adjust2 : public SyncActionNode { public: Adjust2(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class Adjust4 : public SyncActionNode { public: Adjust4(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class Adjust5 : public SyncActionNode { public: Adjust5(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class Adjust3 : public SyncActionNode { public: Adjust3(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class AdjustForPowerShoot : public SyncActionNode { public: AdjustForPowerShoot(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };

class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("min_msec_kick", 500, ""),
            InputPort<double>("msecs_stablize", 1000, ""),
            InputPort<double>("speed_limit", 1.2, ""),
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    Brain *brain;
    rclcpp::Time _startTime;
    string _state = "kick";
    int _msecKick = 1000;
    double _speed;
    double _minRange;
    tuple<double, double, double> _calcSpeed();
};

class Shoot : public StatefulActionNode
{
public:
    Shoot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("msecs", 2000, ""), InputPort<double>("speed", 0.0, "") }; }
    NodeStatus onStart() override { return NodeStatus::SUCCESS; }
    NodeStatus onRunning() override { return NodeStatus::SUCCESS; }
    void onHalted() override {};
private:
    Brain *brain;
};

class RLVisionKick : public StatefulActionNode
{
public:
    RLVisionKick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("max_msec_kick", 10000, "Maximum duration for the kick action (milliseconds)"),
            InputPort<double>("min_msec_kick", 3000, "Minimum duration for the kick action (milliseconds)"),
            InputPort<double>("range", 1.5, "Range within which the strategy is considered effective"),
            InputPort<double>("auto_visual_kick_enable_dist_min", 0.0, "Minimum distance for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_enable_dist_max", 4.0, "Maximum distance for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_enable_angle", 0.785, "Angle range for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_enable_goal_angle", 0.35, "Angle range for enabling auto visual kick towards the goal"),
            InputPort<double>("auto_visual_kick_obstacle_dist_threshold", 1.0, "Distance threshold for obstacles during auto visual kick, if an obstacle is within this distance, auto visual kick will not be executed"),
            InputPort<double>("auto_visual_kick_obstacle_angle_threshold", 1.744, "Angle threshold for obstacles in front during auto visual kick, if an obstacle is within this angle, auto visual kick will not be executed"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;


private:
    Brain *brain;
    rclcpp::Time _startTime;
    rclcpp::Time _headScanStartTime;

    bool _isDecelerating = false;
    bool _visionKickStarted = false;
    bool _pendingRobocupWalk = false;
    rclcpp::Time _decelStartTime;
    motion_policy::VisualKickProgress _progress;
    double _decelDurationMs = 500.0;


    void startDecelerate(double durationMs = 500.0);
    bool stepDecelerate();
};

class Intercept : public StatefulActionNode
{
public:
    Intercept(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("squat_dist", 0.9, ""),
            InputPort<double>("stand_block_dist", 0.2, ""),
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    Brain *brain;
    rclcpp::Time _startTime;
    rclcpp::Time _squatStartTime;
    string _state = "stand";
    string _blockDir = "left";
    double _interceptX, _interceptY;
};

class StandStill : public StatefulActionNode
{
public:
    StandStill(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<int>("msecs", 1000, ""), }; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    Brain *brain;
    rclcpp::Time _startTime;
};


class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<string>("mode", "enter_field", ""),
            InputPort<double>("msecs_interval", 10000, ""),
        };
    };
private:
    Brain *brain;
};

// SelfLocate 变体 
class SelfLocateLine : public SyncActionNode {
public:
    SelfLocateLine(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
    double lineToLineAvgDist(const FieldLine& a, const FieldLineRef& b, int samples = 10);
};

class SelfLocateEnterField : public SyncActionNode {
public:
    SelfLocateEnterField(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() { return { InputPort<double>("msecs_interval", 1000, "") }; }
private:
    Brain *brain;
};

class SelfLocateLocal : public SyncActionNode {
public:
    SelfLocateLocal(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return { InputPort<double>("msecs_interval", 1000, "") };
    };
private:
    Brain *brain;
    bool _singlePenalty();
    bool _doubleX();
};

class SelfLocate1P : public SyncActionNode {
public:
    SelfLocate1P(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocate1M : public SyncActionNode {
public:
    SelfLocate1M(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocate2X : public SyncActionNode {
public:
    SelfLocate2X(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocate2T : public SyncActionNode {
public:
    SelfLocate2T(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocateLT : public SyncActionNode {
public:
    SelfLocateLT(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocatePT : public SyncActionNode {
public:
    SelfLocatePT(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocateBorder : public SyncActionNode {
public:
    SelfLocateBorder(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

// 全位姿校正(含朝向 theta): 中圈与中线的两个 XCross 交点 -> 一次性解出 (x, y, theta)。
// 借鉴 B-Human "center circle with center line" 复合特征, 弥补其余几何校正器"只纠平移不纠朝向"的短板。
// 默认不接入 subtree_locate.xml, 零运行时风险; 验证后再灰度挂入。
class SelfLocateCircle : public SyncActionNode {
public:
    SelfLocateCircle(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<double>("max_theta_drift", 0.6, "朝向最大修正量(rad), 同时拒绝 pi 级翻转"),
        };
    };
private:
    Brain *brain;
};

class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("x", 0, ""), InputPort<double>("y", 0, ""), InputPort<double>("theta", 0, ""),
            InputPort<double>("long_range_threshold", 1.5, ""), InputPort<double>("turn_threshold", 0.4, ""),
            InputPort<double>("vx_limit", 1.0, ""), InputPort<double>("vy_limit", 0.5, ""), InputPort<double>("vtheta_limit", 0.4, ""),
            InputPort<double>("x_tolerance", 0.2, ""), InputPort<double>("y_tolerance", 0.2, ""), InputPort<double>("theta_tolerance", 0.1, ""),
            InputPort<bool>("avoid_obstacle", false, "")
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
};

class GoToReadyPosition : public SyncActionNode
{
public:
    GoToReadyPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("dist_tolerance", 0.5, ""), InputPort<double>("theta_tolerance", 0.1, ""),
            InputPort<double>("vx_limit", 1.2, ""), InputPort<double>("vy_limit", 0.5, ""),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
    motion_policy::ArrivalLock _arrivalLock;
};

// 站位槽位节点 (方案任务4): 计算并前往本机的阵型槽位. scene="auto" 按 GC 状态自动选场景.
// 槽位表/分配算法在 FormationPlanner (formation.h); 本节点负责输入采集(存活/位姿/球位)、冻结与重算、导航执行.
class GoToFormationSlot : public SyncActionNode
{
public:
    GoToFormationSlot(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<string>("scene", "auto", "auto | kickoff_attack | kickoff_defense | freekick_attack | freekick_defense"),
            InputPort<double>("vx_limit", 0.7, ""), InputPort<double>("vy_limit", 0.4, ""),
            InputPort<double>("dist_tolerance", 0.3, ""), InputPort<double>("theta_tolerance", 0.2, ""),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
    // 冻结的分配结果; 场景切换 / 存活集合变化 / 球位移超阈值(带去抖) 时重算
    bool _frozen = false;
    std::string _lastScene;
    std::vector<int> _lastIds;
    Point2D _frozenBall{0.0, 0.0};
    rclcpp::Time _lastReplanTime;
    FormationResult _current;
    motion_policy::ArrivalLock _arrivalLock;
};

class GoToGoalBlockingPosition : public SyncActionNode
{
public:
    GoToGoalBlockingPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("dist_tolerance", 0.5, ""), InputPort<double>("theta_tolerance", 0.1, ""),
            InputPort<double>("vx_limit", 0.5, ""), InputPort<double>("vy_limit", 0.5, ""),
            InputPort<double>("dist_to_goalline", 1.0, ""),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
    motion_policy::ArrivalLock _arrivalLock;
};

class Assist : public SyncActionNode
{
public:
    Assist(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("dist_tolerance", 0.5, ""), InputPort<double>("theta_tolerance", 0.1, ""),
            InputPort<double>("vx_limit", 1.0, ""), InputPort<double>("vy_limit", 0.6, ""),
            InputPort<double>("dist_to_goalline", 1.0, ""),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
};

class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return { InputPort<double>("x", 0, ""), InputPort<double>("y", 0, ""), InputPort<double>("theta", 0, ""), };
    }
private:
    Brain *brain;
};

/**
 * @brief Switch robot to robocup walking mode (kSoccer) and exit VisualKick(false)
 *
 * This node is intended to be executed once when the whole behavior tree starts.
 */
class RobocupWalk : public SyncActionNode
{
public:
    RobocupWalk(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
 
    NodeStatus tick() override;
 
private:
    Brain *brain;
};

class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static PortsList providedPorts() { return {}; }
private:
    Brain *brain;
};


class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { InputPort<string>("action", "start", ""), }; }
private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { InputPort<double>("pitch", 0, ""), InputPort<double>("yaw", 0, ""), }; }
private:
    Brain *brain;
};

class CheckAndStandUp : public SyncActionNode
{
public:
    CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class GoToFreekickPosition : public StatefulActionNode
{
public:
    GoToFreekickPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            // attack_dist 0.7 -> 1.5 (方案任务1): 摆位阶段(GET_READY/SET)开球者也须距球 1.5m, 恢复比赛后才接近开球
            InputPort<string>("side", "attack", ""), InputPort<double>("attack_dist", 1.5, ""),
            InputPort<double>("defense_dist", 1.9, ""), InputPort<double>("vx_limit", 1.2, ""), InputPort<double>("vy_limit", 0.5, ""),
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    Brain *brain;
    bool _isInFinalAdjust = false;
};

class GoBackInField : public SyncActionNode
{
public:
    GoBackInField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("valve", 0.5, ""), }; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

// ------------------------------- FOR DEMO -------------------------------

class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("stop_dist", 1.0, ""), InputPort<double>("stop_angle", 0.1, ""),
            InputPort<double>("vy_limit", 0.4, ""), InputPort<double>("vx_limit", 1.2, ""),
            InputPort<double>("vtheta_limit", 2.4, ""),
        };
    }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class CrabWalk : public SyncActionNode
{
public:
    CrabWalk(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("angle", 0, ""), InputPort<double>("speed", 0.5, ""), }; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class AutoCalibrateVision : public StatefulActionNode
{
public:
    AutoCalibrateVision(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { }; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};
private:
    Brain *brain;
    rclcpp::Time _paramChangeTime;
    vector<vector<double>> _res; 
    double _calcResidual(); 
    int _index; 
    double _subTotal; 
    double _count; 
    double _bestResidual;
};

class CalibrateOdom : public SyncActionNode
{
public:
    CalibrateOdom(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("x", 0, ""), InputPort<double>("y", 0, ""), InputPort<double>("theta", 0, ""), }; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config) {}
    NodeStatus tick() override;
    static PortsList providedPorts() { return {InputPort<std::string>("msg")}; }
private:
    Brain *brain;
};

class PlaySound : public SyncActionNode
{
public:
    PlaySound(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { InputPort<string>("sound", "cheerful", ""), InputPort<bool>("allow_repeat", false, ""), }; }
private:
    Brain *brain;
};

class Speak : public SyncActionNode
{
public:
    Speak(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { InputPort<string>("text", "", ""), }; }
private:
    Brain *brain;
};
