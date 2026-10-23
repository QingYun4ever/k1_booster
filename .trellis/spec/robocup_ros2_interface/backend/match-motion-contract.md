# QYBot referee and motion contracts

## 1. Scope / trigger

Cross-layer contract for `game_controller_interface`, game-controller UDP, brain state, teammate reports and motion output. Applies when changing packet fields, primary/secondary match state, roster, observations or cancellation. Source-of-truth is implemented behavior, not archived strategy comments.

## 2. Signatures

- `game_controller::decodePacket(const uint8_t*, size_t, Packet&) -> DecodeStatus`.
- `Brain::gameControlCallback(const GameControlData&)` commits a validated, own-team, ordered state.
- `Brain::isRecoveryAllowed() const` applies the referee authorization; `isMotionAllowed() const` additionally requires ready recovery state.
- `Brain::isKickAllowed() const` adds fresh local ball, calibrated odometry, lead ownership, in-field ball and restart authorization.
- `RobotClient::stopMotion()` sends zero/cancels active skills even when starting is disallowed. Generic `call` is private.
- `BrainCommunication::update()` runs on the ROS executor thread; socket workers only consume snapshots or enqueue packets.
- `TeammateBallSharing::ingest(const BallReport&, double now_sec)` validates against the current receiver time.

## 3. Contracts

| Boundary | Contract |
|---|---|
| GC v12 | Exact 688 bytes, header RGme, version 12, 11 storage slots |
| Official local GC v20 | Exact 158 bytes, version byte20, raw stopped/gamePhase/setPlay, 20 storage slots |
| ROS | Signed int16 secs_remaining/secondary_time; RobotInfo.available marks roster membership |
| GC return | v12 alive; v20 returnv4 32 bytes, coordinates mm, ball age seconds, angle rad |
| Team wire | QYTM v1 explicit big-endian fields, 64 bytes, confidence0..100; observation ID/age separate from packet ID |
| Local ball | `data->ball` local camera observation/memory only; tmBall never overwrites it |
| Fusion | source_stamp_sec retained through holds; repeated heartbeat/fusion cannot renew age |
| Executor | One ROS executor serializes tick/GC/vision/joystick; bounded socket queue and locked immutable outbound snapshots |
| Deployment | All three robots clean protocol/interface cutover; no old teammate aliases; external GC v12/v20 supported intentionally |

Configuration: team1..254, unique player1..number_of_players, common goalkeeper ID within roster, controller timeout positive. Vision configuration can use `QYBOT_VISION_CONFIG_PATH`; optional system tuning requires `QYBOT_TUNE_SYSTEM=1`. YAML precedence is base, optional local, launch override.

## 4. Validation and error matrix

| Input / condition | Required result |
|---|---|
| Wrong length/header/version/count/team/GK | Decode rejection; no ROS publish |
| Packet for another team or older/duplicate GC packet | No brain state/freshness update |
| StopPlay, SET, penalty, END, timeout/stale GC | No new body movement or skill; cancel existing skill |
| Falling/getting up | No Chase movement; authorized recovery may proceed |
| Uncalibrated/no local fresh ball/nonowner/opponent restart | No kick/VisualKick start |
| NaN/Inf/future observation | No reliable/fresh shared-ball result |
| Duplicate source stamp / heartbeat | No new observation or renewed hold |
| Invalid motion value | Stop/reject rather than serialize nonfinite numbers |
| Missing blackboard entry | Explicit error, never uninitialized primitive |

## 5. Good / base / bad cases

- Good: extra-time corner retains both phase2 and setPlay6; own restart resumes movement/kick; opponent restart permits only defensive positioning.
- Base: packet0 after packet255 is newer; three roster members ignore zero-filled unused storage slots.
- Bad: receiving a heartbeat at time101 for an observation100 must not assign observation stamp101.
- Bad: clearing visual-kick ownership during the deceleration interval before sending cancel permits two active actors.

## 6. Tests required

Run `python tools/check_native.py`: parser negative time/length/counts; source freshness/duplicates/future/reset; match stop/penalty/roster/owner; wire confidence80 and age into real fusion; motion progress distinct observations and arrival-release hysteresis. Release CTest targets must undefine NDEBUG so assert-based native regressions stay active.

On a real robot also verify ROS message regeneration, BTCPP loading, DDS/SDK cancel, three-peer role behavior, point-penalty foot positioning and GC return acceptance. Native tests do not prove these runtime surfaces. A real UDP loopback replay has exercised the parser/match policy; see the migration changelog.

## 7. Wrong versus correct

Wrong: `data->ball.posToField = data->tmBall.posToField;` followed by publishing a local-vision kick.

Correct: choose `getBallForChase()` for approach only; start a kick only when `hasFreshLocalBall()` and `isKickAllowed()` hold.

Wrong: network threads update `brain->tree` or `brain->data` directly.

Correct: enqueue validated packets; apply them in `BrainCommunication::update()` on the serialized executor thread.
