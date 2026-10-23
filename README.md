# QYBot：K1 3v3 足球比赛工程

以 `k1_booster` 为源码基线，迁入近期官方 demo 的 2026 裁判机协议，并修正 3v3 协作、球源时效和动作取消链路。

**三台机器人必须同时部署本工程重新构建的接口与程序。** 队友协议已切换为 `QYTM` v1，不兼容旧工程的队友状态包；裁判机仍支持外部 v12 与官方本地 v20。不要复制旧 `build/install/log`，不要混用两套 ROS 消息。

本机已通过原生 C++ 回归和 UDP 回放；**完整 ROS2/Jetson 构建及机器人动作尚未验证**。本机没有 ROS2/colcon，WSL 因缺少内核组件不能启动。部署前执行下述真机检查。

## 配置三台机器人

运行 `python configure_demo.py` 设置队号、人数、球员号、全队一致的备用守门员号、裁判机地址和 WebUI 地址。

| 配置 | 默认值 | 要求 |
|---|---|---|
| `game.team_id` | 31 | 与裁判机一致，范围 1–254 |
| `game.number_of_players` | 3 | 与实际参赛名单一致 |
| `game.player_id` | 1 | 三台分别配置 1、2、3，不能重复 |
| `game.goalkeeper_id` | 3 | 全队一致；裁判指定编号优先 |
| `game.controller_timeout_ms` | 2000 | 裁判消息失效后禁止身体运动和新技能 |
| `game.field_type` | `adult_size` | 按实测球场选择，不因 3v3 自动缩放 |
| `RLVisionKick.visual_kick_version` | `kV2` | 保留旧工程选择；与目标固件核对，不凭 SDK 目录名判断 |

brain 参数覆盖顺序：`config.yaml → config_local.yaml（存在时）→ launch 参数`。相机标定必须使用各机器人实测配置，不能覆盖成他机标定。

## 离线验证

在工程根目录执行，要求 C++17 编译器、Python 和 PyYAML：

```bash
python tools/check_native.py
python configure_demo.py --help
```

原生检查运行裁判解析、共享球、比赛门控/角色选择、队友协议与融合、动作进展/到位滞回、有效触球时序、动态接应、出球窗口与压力决策测试；同时检查行为树 XML、include/SubTree、配置和 Python 语法。它不替代 ROS 起树、DDS 传输或 SDK 动作验证。

## 出球策略

`Brain::updateKickPlan()` 每帧刷新目标；前锋、守门员和手柄辅助地面踢球使用同一计划。新鲜本机球优先于追球预测和共享球。射门从球位计算方向，扣除机器人圆盘遮挡后选择连续球门窗口；门柱预留 0.2m，射门窗口至少 0.18rad。

| 情况 | 计划与执行 |
|---|---|
| 足够宽的球门窗口 | `shoot`；满足视野和动作 gate 时可启动视觉踢球 |
| 球门窗口不足 | `carry`；选择场内、对已知障碍有净空的前向地面推进目标 |
| 我方禁区危险或对手即将触球 | `clear`；优先短距离前向解围，缩短对位预算 |
| 无合法路线 | `hold`；不沿用旧方向或编造固定出球点 |

地面候选目标距球最多 1.5m；这不是实测球滚动距离或自动传球保证。对手触球时间按 `(距球距离−圆盘半径)/1.2m/s` 估计。视觉踢球要求新鲜球、门柱证据、300ms 内有效凸视野覆盖球到选中目标、朝向/来球线对齐；`/kick_ball` 的方向、目标和距离力度均使用选中目标。首触球保护期间只走地面踢球。

前锋与守门员动作层响应式重评估；地面踢球方向与计划不符时停止。视觉踢球从减速开始保留所有权，取消未成功时不切换到其他身体动作。旧 `CalcKickDir`、位置阈值射门、power-shoot 与独立视觉追球开关已移除；`ball.kick_type` 为 `hold/carry/shoot/clear`。

已执行原生与优化编译回归；真实 BTCPP 4.6.2 控制流 smoke 复现普通 Sequence 漏重评估，并验证响应式动作中止；5000 个固定种子几何场景目标合法且对已知障碍有净空。临时 BTCPP 的 lexy 拼写错误仅在临时副本中修正。未运行完整 ROS brain、DDS、SDK 或实机；未观测障碍、真实出脚误差和球滚动距离仍需实测。

## Ubuntu / Jetson 构建与启动

以下命令需在装有项目依赖的机器人环境执行，未在本机运行：

```bash
source /opt/ros/humble/setup.bash
bash scripts/build.sh
source install/setup.bash
colcon test --packages-select game_controller k1_teammate_ball brain
colcon test-result --verbose
bash scripts/start.sh
```

`scripts/start.sh` 默认使用 `/opt/booster` 中的本机视觉配置。可通过 `QYBOT_VISION_CONFIG_PATH` 指定另一套完整标定目录；性能调优仅在 `QYBOT_TUNE_SYSTEM=1` 时启用。启动不再屏蔽系统更新或杀死无关 Python 进程。停止使用 TERM，让 brain 在 DDS 关闭前取消技能；未正常退出时拒绝重复启动。

## 上场前必须验证

- 行走、视觉踢球中触发 StopPlay、SET、判罚、结束及裁判断流：零速度且技能取消，解除后不会沿用旧技能。
- READY、定位、摔倒恢复、回场：能正常定位；动作 gate 不允许判罚/暂停期间起身或行走。
- 加时角球、普通开球、双方定位球、点球：比赛阶段与定位球类型均保留，Ball Free 前不抢球。
- 主攻者单一；守门员罚下/回场时角色确定；断网使用 GC 编号降级，不按各机局部成本分别抢球。
- 相机停更、近球丢失、未来/异常观测、重新定位：共享球只能引导搜索/接近，不能启动射门。
- 出球目标左右两侧分别放置机器人、靠近边线、我方禁区受压：检查选中路线、地面推进距离和视觉策略真实落点；相机视野停更或遮挡变化后停止旧动作。
- 裁判机收到正确返回包，确认毫米坐标、秒球龄、倒地状态；检查 `/brain/status_json` 的 `protocol_version/stopped/game_phase/set_play/ball_owner_id`。

## 文档

- [本次迁移变更与验证记录](docs/changes/2026-10-21_3v3裁判机与安全链路迁移.md)
- [既有变更索引](docs/changes/README.md)（旧报告描述旧版行为，以本次迁移记录为准）
- [WebUI 部署](webui/README.md)
- [原始官方安装指南](https://zabo9hes6ge.feishu.cn/wiki/OERXwKIZoiJmNJkUe00cM3zvnY6)