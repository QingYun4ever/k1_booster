#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "team_communication_msg.h"
#include "k1_teammate_ball/teammate_ball_sharing.hpp"

int main() {
    TeamCommunicationMsg source;
    source.teamId = 31;
    source.playerId = 2;
    source.playerRole = 1;
    source.communicationId = 20;
    source.ballObservationId = 7;
    source.isAlive = true;
    source.isLead = true;
    source.isInVisualKick = true;
    source.ballDetected = true;
    source.ballLocationKnown = true;
    source.ballObservationAgeMs = 120;
    source.ballConfidence = 80.0f;
    source.ballRange = 1.0f;
    source.ballX = 5.0f;
    source.ballY = -0.5f;
    source.robotX = 4.0f;
    source.robotY = -0.5f;
    source.robotTheta = 0.25f;
    source.kickDir = -0.2f;
    source.cost = 1.25f;
    std::array<uint8_t, team_communication_wire::PACKET_SIZE + 1> bytes{};
    size_t size = 0;
    assert(team_communication_wire::pack(source, bytes.data(), bytes.size(), size));
    TeamCommunicationMsg decoded;
    assert(team_communication_wire::unpack(bytes.data(), size, decoded));
    assert(decoded.ballConfidence == 80.0f && decoded.ballX == 5.0f && decoded.ballY == -0.5f);
    assert(decoded.isInVisualKick && decoded.isLead && decoded.playerId == 2);
    assert(decoded.robotTheta == 0.25f && decoded.kickDir == -0.2f);
    assert(!team_communication_wire::unpack(bytes.data(), size - 1, decoded));
    assert(!team_communication_wire::unpack(bytes.data(), size + 1, decoded));
    bytes[4] = 99;
    assert(!team_communication_wire::unpack(bytes.data(), size, decoded));
    bytes[4] = team_communication_wire::VERSION;
    bytes[20] |= 0x80; // Unknown flags, not a silently accepted extension.
    assert(!team_communication_wire::unpack(bytes.data(), size, decoded));
    assert(team_communication_wire::pack(source, bytes.data(), bytes.size(), size));
    source.ballX = std::numeric_limits<float>::quiet_NaN();
    assert(!team_communication_wire::pack(source, bytes.data(), bytes.size(), size));
    source.ballX = 5.0f;
    source.isAlive = false;
    assert(!team_communication_wire::pack(source, bytes.data(), bytes.size(), size));
    source.isAlive = true;
    source.ballObservationAgeMs = 651;
    assert(!team_communication_wire::pack(source, bytes.data(), bytes.size(), size));
    source.ballObservationAgeMs = 120;
    assert(team_communication_wire::pack(source, bytes.data(), bytes.size(), size));
    assert(team_communication_wire::unpack(bytes.data(), size, decoded));

    assert(team_communication_wire::isNewerSequence(21, 20));
    assert(!team_communication_wire::isNewerSequence(20, 20));
    assert(!team_communication_wire::isNewerSequence(19, 20));
    assert(team_communication_wire::isNewerSequence(1, UINT32_MAX));
    assert(!team_communication_wire::isNewerSequence(0x80000014u, 20));
    uint16_t encodedAge = 0;
    double age = 0;
    assert(team_communication_wire::encodeBallObservationAge(0.650, encodedAge) && encodedAge == 650);
    assert(!team_communication_wire::encodeBallObservationAge(0.651, encodedAge));
    assert(!team_communication_wire::encodeBallObservationAge(-0.01, encodedAge));
    assert(!team_communication_wire::encodeBallObservationAge(NAN, encodedAge));
    assert(team_communication_wire::decodeBallObservationAge(decoded.ballObservationAgeMs, age));

    k1_teammate_ball::TeammateBallSharing sharing;
    k1_teammate_ball::Config config;
    config.enable = true;
    sharing.setConfig(config);
    k1_teammate_ball::BallReport report;
    report.player_id = decoded.playerId;
    report.x = decoded.ballX;
    report.y = decoded.ballY;
    report.confidence = decoded.ballConfidence;
    report.range = decoded.ballRange;
    report.detected = decoded.ballDetected;
    report.location_known = decoded.ballLocationKnown;
    report.robot_x = decoded.robotX;
    report.robot_y = decoded.robotY;
    report.stamp_sec = 100.0 - age;
    sharing.ingest(report, 100.0);
    auto ball = sharing.fuse(0, 0, true, 100.0);
    assert(ball.reliable && ball.is_fresh && ball.source_player_id == 2);
    assert(ball.confidence == 80 && ball.x == 5 && ball.y == -0.5);
    // An unchanged observation heartbeat cannot renew its source timestamp.
    sharing.ingest(report, 100.5);
    ball = sharing.fuse(0, 0, true, 100.5);
    assert(ball.reliable && !ball.is_fresh && ball.source_stamp_sec == report.stamp_sec);
    assert(!sharing.fuse(0, 0, true, 101.0).reliable);
    std::puts("team_wire_test: decode gates, ordering, percent confidence and source-age fusion passed");
}
