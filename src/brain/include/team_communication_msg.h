#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

// Teammate state packets are serialized explicitly. Never send this value type
// directly: its in-memory layout is not part of the network protocol.
struct TeamCommunicationMsg
{
    uint32_t communicationId = 0;
    uint32_t ballObservationId = 0;
    uint16_t teamId = 0;
    uint8_t playerId = 0;
    uint8_t playerRole = 3; // 1: striker, 2: goalkeeper, 3: unknown
    bool isAlive = false;
    bool isLead = false;
    bool ballDetected = false;
    bool ballLocationKnown = false;
    uint16_t ballObservationAgeMs = UINT16_MAX;
    float ballConfidence = 0.f; // vision confidence percentage, 0..100
    float ballRange = 0.f;
    float cost = 0.f;
    float ballX = 0.f;
    float ballY = 0.f;
    float robotX = 0.f;
    float robotY = 0.f;
    float robotTheta = 0.f;
    float kickDir = 0.f;
    float thetaRb = 0.f;
    bool isInVisualKick = false;
};

// Discovery remains on the existing port and wire contract so mixed startup
// and address discovery continue to work. Its all-int payload has no bool or
// compiler padding and is not the teammate-state protocol.
#define VALIDATION_DISCOVERY 41203
struct TeamDiscoveryMsg
{
    int32_t validation = VALIDATION_DISCOVERY;
    int32_t communicationId = 0;
    int32_t teamId = 0;
    int32_t playerId = 0;
};

namespace team_communication_wire
{
// Header: QYTM, version, reserved byte, payload length (big-endian). Payload
// integers/floats are big-endian; positions are meters, angles radians, age ms.
constexpr uint8_t VERSION = 1;
constexpr size_t HEADER_SIZE = 8;
constexpr size_t PAYLOAD_SIZE = 56;
constexpr size_t PACKET_SIZE = HEADER_SIZE + PAYLOAD_SIZE;
constexpr uint16_t NO_BALL_AGE_MS = UINT16_MAX;
constexpr uint16_t MAX_BALL_AGE_MS = 650;
constexpr uint16_t FLAG_ALIVE = 1u << 0;
constexpr uint16_t FLAG_LEAD = 1u << 1;
constexpr uint16_t FLAG_BALL_DETECTED = 1u << 2;
constexpr uint16_t FLAG_BALL_LOCATION_KNOWN = 1u << 3;
constexpr uint16_t FLAG_VISUAL_KICK = 1u << 4;
constexpr uint16_t KNOWN_FLAGS = FLAG_ALIVE | FLAG_LEAD | FLAG_BALL_DETECTED | FLAG_BALL_LOCATION_KNOWN | FLAG_VISUAL_KICK;

inline bool isNewerSequence(uint32_t candidate, uint32_t previous)
{
    const uint32_t delta = candidate - previous;
    return delta != 0 && delta < 0x80000000u;
}

inline bool encodeBallObservationAge(double ageSeconds, uint16_t &ageMs)
{
    if (!std::isfinite(ageSeconds) || ageSeconds < 0.0 || ageSeconds > MAX_BALL_AGE_MS / 1000.0) {
        return false;
    }
    const long long rounded = std::llround(ageSeconds * 1000.0);
    if (rounded < 0 || rounded > MAX_BALL_AGE_MS) return false;
    ageMs = static_cast<uint16_t>(rounded);
    return true;
}

inline bool decodeBallObservationAge(uint16_t ageMs, double &ageSeconds)
{
    if (ageMs == NO_BALL_AGE_MS || ageMs > MAX_BALL_AGE_MS) return false;
    ageSeconds = static_cast<double>(ageMs) / 1000.0;
    return true;
}

inline void put16(uint8_t *out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value >> 8);
    out[1] = static_cast<uint8_t>(value);
}

inline void put32(uint8_t *out, uint32_t value)
{
    out[0] = static_cast<uint8_t>(value >> 24);
    out[1] = static_cast<uint8_t>(value >> 16);
    out[2] = static_cast<uint8_t>(value >> 8);
    out[3] = static_cast<uint8_t>(value);
}

inline uint16_t get16(const uint8_t *in)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
}

inline uint32_t get32(const uint8_t *in)
{
    return (static_cast<uint32_t>(in[0]) << 24) |
           (static_cast<uint32_t>(in[1]) << 16) |
           (static_cast<uint32_t>(in[2]) << 8) |
           static_cast<uint32_t>(in[3]);
}

inline bool valid(const TeamCommunicationMsg &msg)
{
    if (msg.communicationId == 0 || msg.teamId == 0 || msg.teamId > 255 ||
        msg.playerId == 0 || msg.playerId > 11 || msg.playerRole < 1 || msg.playerRole > 3 ||
        (msg.ballLocationKnown && !msg.ballDetected) || (msg.isInVisualKick && !msg.isAlive)) {
        return false;
    }
    if (msg.ballDetected &&
        (msg.ballObservationId == 0 || msg.ballObservationAgeMs == NO_BALL_AGE_MS ||
         msg.ballObservationAgeMs > MAX_BALL_AGE_MS)) {
        return false;
    }
    if (!msg.ballDetected && msg.ballObservationAgeMs != NO_BALL_AGE_MS) {
        return false;
    }
    if (!std::isfinite(msg.ballConfidence) || msg.ballConfidence < 0.f || msg.ballConfidence > 100.f ||
        !std::isfinite(msg.ballRange) || msg.ballRange < 0.f ||
        !std::isfinite(msg.cost) || msg.cost < 0.f ||
        !std::isfinite(msg.ballX) || !std::isfinite(msg.ballY) ||
        !std::isfinite(msg.robotX) || !std::isfinite(msg.robotY) ||
        !std::isfinite(msg.robotTheta) || !std::isfinite(msg.kickDir) || !std::isfinite(msg.thetaRb)) {
        return false;
    }
    return true;
}

inline void putFloat(uint8_t *out, float value)
{
    static_assert(sizeof(float) == sizeof(uint32_t), "wire protocol requires IEEE-754 binary32");
    static_assert(std::numeric_limits<float>::is_iec559, "wire protocol requires IEEE-754 binary32");
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    put32(out, bits);
}

inline float getFloat(const uint8_t *in)
{
    uint32_t bits = get32(in);
    float value = 0.f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline bool pack(const TeamCommunicationMsg &msg, uint8_t *out, size_t capacity, size_t &written)
{
    written = 0;
    if (out == nullptr || capacity < PACKET_SIZE || !valid(msg)) {
        return false;
    }

    out[0] = 'Q';
    out[1] = 'Y';
    out[2] = 'T';
    out[3] = 'M';
    out[4] = VERSION;
    out[5] = 0;
    put16(out + 6, static_cast<uint16_t>(PAYLOAD_SIZE));

    size_t offset = HEADER_SIZE;
    put32(out + offset, msg.communicationId); offset += 4;
    put32(out + offset, msg.ballObservationId); offset += 4;
    put16(out + offset, msg.teamId); offset += 2;
    out[offset++] = msg.playerId;
    out[offset++] = msg.playerRole;
    uint16_t flags = 0;
    if (msg.isAlive) flags |= FLAG_ALIVE;
    if (msg.isLead) flags |= FLAG_LEAD;
    if (msg.ballDetected) flags |= FLAG_BALL_DETECTED;
    if (msg.ballLocationKnown) flags |= FLAG_BALL_LOCATION_KNOWN;
    if (msg.isInVisualKick) flags |= FLAG_VISUAL_KICK;
    put16(out + offset, flags); offset += 2;
    put16(out + offset, msg.ballObservationAgeMs); offset += 2;
    putFloat(out + offset, msg.ballConfidence); offset += 4;
    putFloat(out + offset, msg.ballRange); offset += 4;
    putFloat(out + offset, msg.cost); offset += 4;
    putFloat(out + offset, msg.ballX); offset += 4;
    putFloat(out + offset, msg.ballY); offset += 4;
    putFloat(out + offset, msg.robotX); offset += 4;
    putFloat(out + offset, msg.robotY); offset += 4;
    putFloat(out + offset, msg.robotTheta); offset += 4;
    putFloat(out + offset, msg.kickDir); offset += 4;
    putFloat(out + offset, msg.thetaRb); offset += 4;
    if (offset != PACKET_SIZE) {
        return false;
    }
    written = offset;
    return true;
}

inline bool unpack(const uint8_t *packet, size_t length, TeamCommunicationMsg &msg)
{
    if (packet == nullptr || length != PACKET_SIZE ||
        packet[0] != 'Q' || packet[1] != 'Y' || packet[2] != 'T' || packet[3] != 'M' ||
        packet[4] != VERSION || packet[5] != 0 || get16(packet + 6) != PAYLOAD_SIZE) {
        return false;
    }

    TeamCommunicationMsg decoded;
    size_t offset = HEADER_SIZE;
    decoded.communicationId = get32(packet + offset); offset += 4;
    decoded.ballObservationId = get32(packet + offset); offset += 4;
    decoded.teamId = get16(packet + offset); offset += 2;
    decoded.playerId = packet[offset++];
    decoded.playerRole = packet[offset++];
    const uint16_t flags = get16(packet + offset); offset += 2;
    if ((flags & ~KNOWN_FLAGS) != 0) {
        return false;
    }
    decoded.isAlive = (flags & FLAG_ALIVE) != 0;
    decoded.isLead = (flags & FLAG_LEAD) != 0;
    decoded.ballDetected = (flags & FLAG_BALL_DETECTED) != 0;
    decoded.ballLocationKnown = (flags & FLAG_BALL_LOCATION_KNOWN) != 0;
    decoded.ballObservationAgeMs = get16(packet + offset); offset += 2;
    decoded.ballConfidence = getFloat(packet + offset); offset += 4;
    decoded.ballRange = getFloat(packet + offset); offset += 4;
    decoded.cost = getFloat(packet + offset); offset += 4;
    decoded.ballX = getFloat(packet + offset); offset += 4;
    decoded.ballY = getFloat(packet + offset); offset += 4;
    decoded.robotX = getFloat(packet + offset); offset += 4;
    decoded.robotY = getFloat(packet + offset); offset += 4;
    decoded.robotTheta = getFloat(packet + offset); offset += 4;
    decoded.kickDir = getFloat(packet + offset); offset += 4;
    decoded.thetaRb = getFloat(packet + offset); offset += 4;
    decoded.isInVisualKick = (flags & FLAG_VISUAL_KICK) != 0;
    if (offset != length || !valid(decoded)) {
        return false;
    }
    msg = decoded;
    return true;
}
} // namespace team_communication_wire
