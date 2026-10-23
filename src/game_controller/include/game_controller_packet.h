#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "game_controller_protocol_v20.h"
#include "RoboCupGameControlData.h"

namespace game_controller
{

constexpr std::uint8_t kLegacyVersion = HL_GAMECONTROLLER_STRUCT_VERSION;
constexpr std::uint8_t kOfficialVersion = game_controller_protocol_v20::kStructVersion;
constexpr std::size_t kLegacyMaxPlayers = HL_MAX_NUM_PLAYERS;
constexpr std::size_t kOfficialMaxPlayers = game_controller_protocol_v20::kMaxNumPlayers;
constexpr std::size_t kCoachMessageSize = SPL_COACH_MESSAGE_SIZE;
constexpr std::size_t kLegacyTeamSize = sizeof(HlTeamInfo);
constexpr std::size_t kLegacyPacketSize = sizeof(HlRoboCupGameControlData);
constexpr std::size_t kOfficialTeamSize = sizeof(game_controller_protocol_v20::TeamInfo);
constexpr std::size_t kOfficialPacketSize = sizeof(game_controller_protocol_v20::GameControlData);
constexpr std::size_t kMaxPacketSize = kLegacyPacketSize;

static_assert(sizeof(HlRobotInfo) == 6, "GameController v12 RobotInfo layout mismatch");
static_assert(kLegacyTeamSize == 332, "GameController v12 TeamInfo layout mismatch");
static_assert(kLegacyPacketSize == 688, "GameController v12 packet layout mismatch");
static_assert(offsetof(HlRoboCupGameControlData, teams) == 24, "GameController v12 team offset mismatch");
static_assert(offsetof(HlTeamInfo, coachMessage) == 7, "GameController v12 coach message offset mismatch");
static_assert(offsetof(HlTeamInfo, coach) == 260, "GameController v12 coach offset mismatch");
static_assert(offsetof(HlTeamInfo, players) == 266, "GameController v12 players offset mismatch");

struct RobotInfo
{
    std::uint8_t penalty = 0;
    std::uint8_t secsTillUnpenalised = 0;
    std::uint8_t numberOfWarnings = 0;
    std::uint8_t yellowCardCount = 0;
    std::uint8_t redCardCount = 0;
    std::uint8_t goalKeeper = 0;
    std::uint8_t cautions = 0;
    bool available = false;
};

struct TeamInfo
{
    std::uint8_t teamNumber = 0;
    std::uint8_t fieldPlayerColour = 0;
    std::uint8_t goalkeeperColour = 0;
    std::uint8_t goalkeeper = 0;
    std::uint8_t score = 0;
    std::uint8_t penaltyShot = 0;
    std::uint16_t singleShots = 0;
    std::uint16_t messageBudget = 0;
    std::uint8_t coachSequence = 0;
    std::array<std::uint8_t, kCoachMessageSize> coachMessage{};
    RobotInfo coach{};
    std::array<RobotInfo, kOfficialMaxPlayers> players{};
};

struct Packet
{
    bool legacy = false;
    std::uint16_t version = 0;
    std::uint8_t packetNumber = 0;
    std::uint8_t playersPerTeam = 0;

    // v12-only values are kept separate from the v20 fields.
    std::uint8_t gameType = 0;
    std::uint8_t secondaryState = 0;
    std::array<std::uint8_t, 4> secondaryStateInfo{};
    std::uint8_t dropInTeam = 0;
    std::uint16_t dropInTime = 0;

    // v20 local GameController fields, retained in their wire representation.
    std::uint8_t competitionType = 0;
    std::uint8_t stopped = 0;
    std::uint8_t gamePhase = 0;
    std::uint8_t state = 0;
    std::uint8_t setPlay = 0;
    std::uint8_t firstHalf = 0;
    std::uint8_t kickingTeam = 0;
    std::int16_t secsRemaining = 0;
    std::int16_t secondaryTime = 0;

    std::array<TeamInfo, 2> teams{};
};

enum class DecodeStatus
{
    Ok,
    InvalidLength,
    InvalidHeader,
    UnsupportedVersion,
    InvalidCount,
    InvalidState,
    InvalidPhase,
    InvalidSetPlay,
    InvalidTeam,
    InvalidGoalkeeper,
};

inline std::uint16_t readU16LE(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(data[0])
        | static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

inline std::int16_t readI16LE(const std::uint8_t *data)
{
    const std::uint16_t bits = readU16LE(data);
    const std::int32_t value = bits <= 0x7fffU
        ? static_cast<std::int32_t>(bits)
        : static_cast<std::int32_t>(bits) - 0x10000;
    return static_cast<std::int16_t>(value);
}

inline bool isKnownState(std::uint8_t state, bool legacy)
{
    return state <= STATE_FINISHED || (legacy && state == STATE_DANIEL);
}
// The v12-specific STATE_DANIEL value is not a standard primary state; fail closed as INITIAL/stopped.
inline bool legacyStateRequiresStop(std::uint8_t state)
{
    return state == STATE_DANIEL;
}

inline std::uint8_t legacyStateForRos(std::uint8_t state)
{
    return legacyStateRequiresStop(state) ? STATE_INITIAL : state;
}

inline bool isKnownColour(std::uint8_t colour)
{
    return colour <= 9;
}


inline bool teamsHaveValidNumbers(const Packet &packet)
{
    return packet.teams[0].teamNumber != 0
        && packet.teams[1].teamNumber != 0
        && packet.teams[0].teamNumber != packet.teams[1].teamNumber;
}

inline bool hasValidKickingTeam(const Packet &packet)
{
    return packet.kickingTeam == game_controller_protocol_v20::kKickingTeamNone
        || packet.kickingTeam == packet.teams[0].teamNumber
        || packet.kickingTeam == packet.teams[1].teamNumber;
}

inline DecodeStatus decodeLegacy(const std::uint8_t *bytes, Packet &packet)
{
    const std::size_t versionOffset = offsetof(HlRoboCupGameControlData, version);
    if (readU16LE(bytes + versionOffset) != kLegacyVersion)
    {
        return DecodeStatus::UnsupportedVersion;
    }

    const std::uint8_t playersPerTeam = bytes[offsetof(HlRoboCupGameControlData, playersPerTeam)];
    const std::uint8_t state = bytes[offsetof(HlRoboCupGameControlData, state)];
    const std::uint8_t firstHalf = bytes[offsetof(HlRoboCupGameControlData, firstHalf)];
    const std::uint8_t secondaryState = bytes[offsetof(HlRoboCupGameControlData, secondaryState)];
    if (playersPerTeam == 0 || playersPerTeam > kLegacyMaxPlayers)
    {
        return DecodeStatus::InvalidCount;
    }
    if (!isKnownState(state, true))
    {
        return DecodeStatus::InvalidState;
    }
    if (firstHalf > 1 || secondaryState > 9)
    {
        return DecodeStatus::InvalidPhase;
    }

    packet = Packet{};
    packet.legacy = true;
    packet.version = kLegacyVersion;
    packet.packetNumber = bytes[offsetof(HlRoboCupGameControlData, packetNumber)];
    packet.playersPerTeam = playersPerTeam;
    packet.gameType = bytes[offsetof(HlRoboCupGameControlData, gameType)];
    packet.state = state;
    packet.firstHalf = firstHalf;
    packet.kickingTeam = bytes[offsetof(HlRoboCupGameControlData, kickOffTeam)];
    packet.secondaryState = secondaryState;
    const std::size_t secondaryInfoOffset = offsetof(HlRoboCupGameControlData, secondaryStateInfo);
    for (std::size_t i = 0; i < packet.secondaryStateInfo.size(); ++i)
    {
        packet.secondaryStateInfo[i] = bytes[secondaryInfoOffset + i];
    }
    packet.dropInTeam = bytes[offsetof(HlRoboCupGameControlData, dropInTeam)];
    packet.dropInTime = readU16LE(bytes + offsetof(HlRoboCupGameControlData, dropInTime));
    packet.secsRemaining = readI16LE(bytes + offsetof(HlRoboCupGameControlData, secsRemaining));
    packet.secondaryTime = readI16LE(bytes + offsetof(HlRoboCupGameControlData, secondaryTime));

    const std::size_t teamsOffset = offsetof(HlRoboCupGameControlData, teams);
    for (std::size_t teamIndex = 0; teamIndex < packet.teams.size(); ++teamIndex)
    {
        const std::size_t offset = teamsOffset + teamIndex * kLegacyTeamSize;
        TeamInfo &team = packet.teams[teamIndex];
        team.teamNumber = bytes[offset + offsetof(HlTeamInfo, teamNumber)];
        team.fieldPlayerColour = bytes[offset + offsetof(HlTeamInfo, fieldPlayerColour)];
        team.score = bytes[offset + offsetof(HlTeamInfo, score)];
        team.penaltyShot = bytes[offset + offsetof(HlTeamInfo, penaltyShot)];
        team.singleShots = readU16LE(bytes + offset + offsetof(HlTeamInfo, singleShots));
        team.coachSequence = bytes[offset + offsetof(HlTeamInfo, coachSequence)];

        const std::size_t coachMessageOffset = offset + offsetof(HlTeamInfo, coachMessage);
        for (std::size_t i = 0; i < team.coachMessage.size(); ++i)
        {
            team.coachMessage[i] = bytes[coachMessageOffset + i];
        }

        const std::size_t coachOffset = offset + offsetof(HlTeamInfo, coach);
        team.coach.penalty = bytes[coachOffset + offsetof(HlRobotInfo, penalty)];
        team.coach.secsTillUnpenalised = bytes[coachOffset + offsetof(HlRobotInfo, secsTillUnpenalised)];
        team.coach.numberOfWarnings = bytes[coachOffset + offsetof(HlRobotInfo, numberOfWarnings)];
        team.coach.yellowCardCount = bytes[coachOffset + offsetof(HlRobotInfo, yellowCardCount)];
        team.coach.redCardCount = bytes[coachOffset + offsetof(HlRobotInfo, redCardCount)];
        team.coach.goalKeeper = bytes[coachOffset + offsetof(HlRobotInfo, goalKeeper)];
        team.coach.available = true;

        const std::size_t playersOffset = offset + offsetof(HlTeamInfo, players);
        for (std::size_t playerIndex = 0; playerIndex < kLegacyMaxPlayers; ++playerIndex)
        {
            const std::size_t playerOffset = playersOffset + playerIndex * sizeof(HlRobotInfo);
            RobotInfo &player = team.players[playerIndex];
            player.penalty = bytes[playerOffset + offsetof(HlRobotInfo, penalty)];
            player.secsTillUnpenalised = bytes[playerOffset + offsetof(HlRobotInfo, secsTillUnpenalised)];
            player.numberOfWarnings = bytes[playerOffset + offsetof(HlRobotInfo, numberOfWarnings)];
            player.yellowCardCount = bytes[playerOffset + offsetof(HlRobotInfo, yellowCardCount)];
            player.redCardCount = bytes[playerOffset + offsetof(HlRobotInfo, redCardCount)];
            player.goalKeeper = bytes[playerOffset + offsetof(HlRobotInfo, goalKeeper)];
            player.available = playerIndex < playersPerTeam;
        }

        if (!isKnownColour(team.fieldPlayerColour) || team.coach.goalKeeper > 1)
        {
            return team.coach.goalKeeper > 1
                ? DecodeStatus::InvalidGoalkeeper
                : DecodeStatus::InvalidTeam;
        }
        for (std::size_t playerIndex = 0; playerIndex < kLegacyMaxPlayers; ++playerIndex)
        {
            if (team.players[playerIndex].goalKeeper > 1)
            {
                return DecodeStatus::InvalidGoalkeeper;
            }
            if (team.players[playerIndex].goalKeeper != 0)
            {
                if (playerIndex >= playersPerTeam || team.goalkeeper != 0)
                {
                    return DecodeStatus::InvalidGoalkeeper;
                }
                team.goalkeeper = static_cast<std::uint8_t>(playerIndex + 1);
            }
        }
    }

    return teamsHaveValidNumbers(packet) && hasValidKickingTeam(packet)
        ? DecodeStatus::Ok
        : DecodeStatus::InvalidTeam;
}

inline DecodeStatus decodeOfficial(const std::uint8_t *bytes, Packet &packet)
{
    using OfficialWire = game_controller_protocol_v20::GameControlData;
    using OfficialTeamWire = game_controller_protocol_v20::TeamInfo;
    using OfficialRobotWire = game_controller_protocol_v20::RobotInfo;

    const std::uint8_t playersPerTeam = bytes[offsetof(OfficialWire, playersPerTeam)];
    const std::uint8_t competitionType = bytes[offsetof(OfficialWire, competitionType)];
    const std::uint8_t stopped = bytes[offsetof(OfficialWire, stopped)];
    const std::uint8_t gamePhase = bytes[offsetof(OfficialWire, gamePhase)];
    const std::uint8_t state = bytes[offsetof(OfficialWire, state)];
    const std::uint8_t setPlay = bytes[offsetof(OfficialWire, setPlay)];
    const std::uint8_t firstHalf = bytes[offsetof(OfficialWire, firstHalf)];
    if (playersPerTeam == 0 || playersPerTeam > kOfficialMaxPlayers)
    {
        return DecodeStatus::InvalidCount;
    }
    if (!isKnownState(state, false))
    {
        return DecodeStatus::InvalidState;
    }
    if (competitionType > 1 || stopped > 1
        || gamePhase > game_controller_protocol_v20::kGamePhaseTimeout || firstHalf > 1)
    {
        return DecodeStatus::InvalidPhase;
    }
    if (setPlay > game_controller_protocol_v20::kSetPlayCornerKick)
    {
        return DecodeStatus::InvalidSetPlay;
    }

    packet = Packet{};
    packet.version = kOfficialVersion;
    packet.packetNumber = bytes[offsetof(OfficialWire, packetNumber)];
    packet.playersPerTeam = playersPerTeam;
    packet.competitionType = competitionType;
    packet.stopped = stopped;
    packet.gamePhase = gamePhase;
    packet.state = state;
    packet.setPlay = setPlay;
    packet.firstHalf = firstHalf;
    packet.kickingTeam = bytes[offsetof(OfficialWire, kickingTeam)];
    packet.secsRemaining = readI16LE(bytes + offsetof(OfficialWire, secsRemaining));
    packet.secondaryTime = readI16LE(bytes + offsetof(OfficialWire, secondaryTime));

    const std::size_t teamsOffset = offsetof(OfficialWire, teams);
    for (std::size_t teamIndex = 0; teamIndex < packet.teams.size(); ++teamIndex)
    {
        const std::size_t offset = teamsOffset + teamIndex * sizeof(OfficialTeamWire);
        TeamInfo &team = packet.teams[teamIndex];
        team.teamNumber = bytes[offset + offsetof(OfficialTeamWire, teamNumber)];
        team.fieldPlayerColour = bytes[offset + offsetof(OfficialTeamWire, fieldPlayerColour)];
        team.goalkeeperColour = bytes[offset + offsetof(OfficialTeamWire, goalkeeperColour)];
        team.goalkeeper = bytes[offset + offsetof(OfficialTeamWire, goalkeeper)];
        team.score = bytes[offset + offsetof(OfficialTeamWire, score)];
        team.penaltyShot = bytes[offset + offsetof(OfficialTeamWire, penaltyShot)];
        team.singleShots = readU16LE(bytes + offset + offsetof(OfficialTeamWire, singleShots));
        team.messageBudget = readU16LE(bytes + offset + offsetof(OfficialTeamWire, messageBudget));

        if (!isKnownColour(team.fieldPlayerColour) || !isKnownColour(team.goalkeeperColour))
        {
            return DecodeStatus::InvalidTeam;
        }
        if (team.goalkeeper > playersPerTeam)
        {
            return DecodeStatus::InvalidGoalkeeper;
        }

        const std::size_t playersOffset = offset + offsetof(OfficialTeamWire, players);
        for (std::size_t playerIndex = 0; playerIndex < kOfficialMaxPlayers; ++playerIndex)
        {
            const std::size_t playerOffset = playersOffset + playerIndex * sizeof(OfficialRobotWire);
            RobotInfo &player = team.players[playerIndex];
            player.penalty = bytes[playerOffset + offsetof(OfficialRobotWire, penalty)];
            player.secsTillUnpenalised = bytes[playerOffset + offsetof(OfficialRobotWire, secsTillUnpenalised)];
            player.cautions = bytes[playerOffset + offsetof(OfficialRobotWire, cautions)];
            player.available = playerIndex < playersPerTeam;
        }
    }

    return teamsHaveValidNumbers(packet) && hasValidKickingTeam(packet)
        ? DecodeStatus::Ok
        : DecodeStatus::InvalidTeam;
}

inline DecodeStatus decodePacket(const std::uint8_t *bytes, std::size_t size, Packet &packet)
{
    if (bytes == nullptr || size < 4)
    {
        return DecodeStatus::InvalidLength;
    }
    if (bytes[0] != 'R' || bytes[1] != 'G' || bytes[2] != 'm' || bytes[3] != 'e')
    {
        return DecodeStatus::InvalidHeader;
    }
    if (size < 5)
    {
        return DecodeStatus::InvalidLength;
    }

    if (bytes[4] == kLegacyVersion)
    {
        if (size != kLegacyPacketSize)
        {
            return DecodeStatus::InvalidLength;
        }
        return decodeLegacy(bytes, packet);
    }
    if (bytes[4] == kOfficialVersion)
    {
        if (size != kOfficialPacketSize)
        {
            return DecodeStatus::InvalidLength;
        }
        return decodeOfficial(bytes, packet);
    }
    return DecodeStatus::UnsupportedVersion;
}

} // namespace game_controller
