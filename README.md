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

原生检查运行裁判解析、共享球、比赛门控/角色选择、队友协议与融合、动作进展/到位滞回，以及有效触球时序测试；同时检查行为树 XML、include/SubTree、配置和 Python 语法。它不替代 ROS 起树、DDS 传输或 SDK 动作验证。

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
- 裁判机收到正确返回包，确认毫米坐标、秒球龄、倒地状态；检查 `/brain/status_json` 的 `protocol_version/stopped/game_phase/set_play/ball_owner_id`。

## 文档

- [本次迁移变更与验证记录](docs/changes/2026-10-21_3v3裁判机与安全链路迁移.md)
- [既有变更索引](docs/changes/README.md)（旧报告描述旧版行为，以本次迁移记录为准）
- [WebUI 部署](webui/README.md)
- [原始官方安装指南](https://zabo9hes6ge.feishu.cn/wiki/OERXwKIZoiJmNJkUe00cM3zvnY6)