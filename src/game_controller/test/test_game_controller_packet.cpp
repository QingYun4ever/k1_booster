// Standalone parser tests; no ROS dependency is required.
// g++ -std=c++17 -Isrc/game_controller/include src/game_controller/test/test_game_controller_packet.cpp -o /tmp/test_game_controller_packet && /tmp/test_game_controller_packet

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "game_controller_packet.h"

namespace
{
using game_controller::DecodeStatus;
using game_controller::Packet;

void putU16(std::vector<std::uint8_t> &bytes, std::size_t offset, std::uint16_t value)
{
    bytes[offset] = static_cast<std::uint8_t>(value & 0xffU);
    bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

void putI16(std::vector<std::uint8_t> &bytes, std::size_t offset, std::int16_t value)
{
    putU16(bytes, offset, static_cast<std::uint16_t>(value));
}

std::vector<std::uint8_t> makeOfficialPacket()
{
    std::vector<std::uint8_t> bytes(game_controller::kOfficialPacketSize, 0);
    bytes[0] = 'R';
    bytes[1] = 'G';
    bytes[2] = 'm';
    bytes[3] = 'e';
    bytes[4] = 20;
    bytes[5] = 0xa5;
    bytes[6] = 5;
    bytes[7] = 1;
    bytes[8] = 1;
    bytes[9] = 2;
    bytes[10] = 3;
    bytes[11] = 6;
    bytes[12] = 0;
    bytes[13] = 42;
    putI16(bytes, 14, -1);
    putI16(bytes, 16, -32768);

    const std::size_t firstTeam = 18;
    bytes[firstTeam] = 42;
    bytes[firstTeam + 1] = 0;
    bytes[firstTeam + 2] = 1;
    bytes[firstTeam + 3] = 2;
    bytes[firstTeam + 4] = 3;
    bytes[firstTeam + 5] = 4;
    putU16(bytes, firstTeam + 6, 0x1234);
    putU16(bytes, firstTeam + 8, 0x5678);
    bytes[firstTeam + 10] = 7;
    bytes[firstTeam + 11] = 8;
    bytes[firstTeam + 12] = 9;
    const std::size_t lastPlayer = firstTeam + 10 + 19 * 3;
    bytes[lastPlayer] = 10;
    bytes[lastPlayer + 1] = 11;
    bytes[lastPlayer + 2] = 12;

    const std::size_t secondTeam = firstTeam + game_controller::kOfficialTeamSize;
    bytes[secondTeam] = 43;
    bytes[secondTeam + 1] = 2;
    bytes[secondTeam + 2] = 3;
    bytes[secondTeam + 3] = 5;
    return bytes;
}

std::vector<std::uint8_t> makeLegacyPacket()
{
    std::vector<std::uint8_t> bytes(game_controller::kLegacyPacketSize, 0);
    bytes[0] = 'R';
    bytes[1] = 'G';
    bytes[2] = 'm';
    bytes[3] = 'e';
    putU16(bytes, 4, 12);
    bytes[6] = 8;
    bytes[7] = 5;
    bytes[8] = 2;
    bytes[9] = 3;
    bytes[10] = 1;
    bytes[11] = 13;
    bytes[12] = 8;
    bytes[13] = 13;
    bytes[14] = 1;
    bytes[15] = 99;
    bytes[16] = 7;
    bytes[17] = 255;
    putU16(bytes, 18, 0xffff);
    putU16(bytes, 20, 300);
    putU16(bytes, 22, 25);

    const std::size_t firstTeam = 24;
    bytes[firstTeam] = 13;
    bytes[firstTeam + 1] = 0;
    bytes[firstTeam + 2] = 2;
    bytes[firstTeam + 3] = 1;
    putU16(bytes, firstTeam + 4, 0x1234);
    bytes[firstTeam + 6] = 9;
    bytes[firstTeam + 7] = 0x55;
    bytes[firstTeam + 7 + game_controller::kCoachMessageSize - 1] = 0xaa;
    const std::size_t firstPlayer = firstTeam + 266;
    bytes[firstPlayer] = 4;
    bytes[firstPlayer + 1] = 5;
    bytes[firstPlayer + 2] = 6;
    bytes[firstPlayer + 3] = 7;
    bytes[firstPlayer + 4] = 8;
    bytes[firstPlayer + 5] = 0;
    bytes[firstPlayer + 6 + 5] = 1;

    const std::size_t secondTeam = firstTeam + game_controller::kLegacyTeamSize;
    bytes[secondTeam] = 14;
    bytes[secondTeam + 1] = 1;
    bytes[secondTeam + 2] = 4;
    return bytes;
}

void expectDecodeFailure(const std::vector<std::uint8_t> &bytes, DecodeStatus expected)
{
    Packet packet{};
    assert(game_controller::decodePacket(bytes.data(), bytes.size(), packet) == expected);
}

void testOfficialDecodeAndSignedTimes()
{
    const std::vector<std::uint8_t> bytes = makeOfficialPacket();
    Packet packet{};
    assert(game_controller::decodePacket(bytes.data(), bytes.size(), packet) == DecodeStatus::Ok);
    assert(!packet.legacy);
    assert(packet.version == 20);
    assert(packet.packetNumber == 0xa5);
    assert(packet.playersPerTeam == 5);
    assert(packet.competitionType == 1);
    assert(packet.stopped == 1);
    assert(packet.gamePhase == 2);
    assert(packet.state == 3);
    assert(packet.setPlay == 6);
    assert(packet.firstHalf == 0);
    assert(packet.kickingTeam == 42);
    assert(packet.secsRemaining == -1);
    assert(packet.secondaryTime == -32768);
    assert(packet.teams[0].teamNumber == 42);
    assert(packet.teams[0].goalkeeperColour == 1);
    assert(packet.teams[0].goalkeeper == 2);
    assert(packet.teams[0].singleShots == 0x1234);
    assert(packet.teams[0].messageBudget == 0x5678);
    assert(packet.teams[0].players[0].penalty == 7);
    assert(packet.teams[0].players[0].secsTillUnpenalised == 8);
    assert(packet.teams[0].players[0].cautions == 9);
    assert(packet.teams[0].players[0].available);
    assert(packet.teams[0].players[4].available);
    assert(!packet.teams[0].players[5].available);
    assert(!packet.teams[0].players[19].available);
    assert(packet.teams[0].players[19].penalty == 10);
    assert(packet.teams[0].players[19].cautions == 12);
    assert(packet.teams[1].teamNumber == 43);
    assert(game_controller_protocol_v20::toLegacySecondaryState(packet.gamePhase, packet.setPlay) == 2);
    assert(game_controller_protocol_v20::toLegacySecondaryState(0, 4) == 9);
    assert(game_controller_protocol_v20::toLegacySecondaryState(0, 6) == 7);
}

void testLegacyDecode()
{
    const std::vector<std::uint8_t> bytes = makeLegacyPacket();
    Packet packet{};
    assert(game_controller::decodePacket(bytes.data(), bytes.size(), packet) == DecodeStatus::Ok);
    assert(packet.legacy);
    assert(packet.version == 12);
    assert(packet.packetNumber == 8);
    assert(packet.playersPerTeam == 5);
    assert(packet.gameType == 2);
    assert(packet.state == 3);
    assert(packet.firstHalf == 1);
    assert(packet.kickingTeam == 13);
    assert(packet.secondaryState == 8);
    assert(packet.secondaryStateInfo[0] == 13);
    assert(packet.secondaryStateInfo[1] == 1);
    assert(packet.dropInTeam == 255);
    assert(packet.dropInTime == 0xffff);
    assert(packet.secsRemaining == 300);
    assert(packet.secondaryTime == 25);
    assert(packet.teams[0].coachSequence == 9);
    assert(packet.teams[0].coachMessage[0] == 0x55);
    assert(packet.teams[0].coachMessage.back() == 0xaa);
    assert(packet.teams[0].singleShots == 0x1234);
    assert(packet.teams[0].players[0].penalty == 4);
    assert(packet.teams[0].players[1].goalKeeper == 1);
    assert(packet.teams[0].players[4].available);
    assert(!packet.teams[0].players[5].available);
    assert(packet.teams[0].goalkeeper == 2);
    assert(packet.teams[1].teamNumber == 14);
}

void testOfficialBoundariesAndValidation()
{
    const std::vector<std::uint8_t> valid = makeOfficialPacket();
    Packet packet{};
    assert(game_controller::decodePacket(valid.data(), valid.size(), packet) == DecodeStatus::Ok);
    auto boundary = valid;
    boundary[6] = static_cast<std::uint8_t>(game_controller::kOfficialMaxPlayers);
    assert(game_controller::decodePacket(boundary.data(), boundary.size(), packet) == DecodeStatus::Ok);
    assert(packet.playersPerTeam == game_controller::kOfficialMaxPlayers);
    assert(packet.teams[0].players[19].available);
    boundary = valid;
    boundary[13] = game_controller_protocol_v20::kKickingTeamNone;
    assert(game_controller::decodePacket(boundary.data(), boundary.size(), packet) == DecodeStatus::Ok);
    assert(packet.kickingTeam == game_controller_protocol_v20::kKickingTeamNone);

    auto bytes = valid;
    bytes.resize(valid.size() - 1);
    expectDecodeFailure(bytes, DecodeStatus::InvalidLength);
    bytes = valid;
    bytes.push_back(0);
    expectDecodeFailure(bytes, DecodeStatus::InvalidLength);
    bytes = valid;
    bytes[0] = 'X';
    expectDecodeFailure(bytes, DecodeStatus::InvalidHeader);
    bytes = valid;
    bytes[4] = 19;
    expectDecodeFailure(bytes, DecodeStatus::UnsupportedVersion);
    bytes = valid;
    bytes[6] = 0;
    expectDecodeFailure(bytes, DecodeStatus::InvalidCount);
    bytes = valid;
    bytes[6] = 21;
    expectDecodeFailure(bytes, DecodeStatus::InvalidCount);
    bytes = valid;
    bytes[10] = 5;
    expectDecodeFailure(bytes, DecodeStatus::InvalidState);
    bytes = valid;
    bytes[7] = 2;
    expectDecodeFailure(bytes, DecodeStatus::InvalidPhase);
    bytes = valid;
    bytes[8] = 2;
    expectDecodeFailure(bytes, DecodeStatus::InvalidPhase);
    bytes = valid;
    bytes[9] = 4;
    expectDecodeFailure(bytes, DecodeStatus::InvalidPhase);
    bytes = valid;
    bytes[11] = 7;
    expectDecodeFailure(bytes, DecodeStatus::InvalidSetPlay);
    bytes = valid;
    bytes[12] = 2;
    expectDecodeFailure(bytes, DecodeStatus::InvalidPhase);
    bytes = valid;
    bytes[13] = 44;
    expectDecodeFailure(bytes, DecodeStatus::InvalidTeam);
    bytes = valid;
    bytes[18 + game_controller::kOfficialTeamSize] = 42;
    expectDecodeFailure(bytes, DecodeStatus::InvalidTeam);
    bytes = valid;
    bytes[18 + 1] = 10;
    expectDecodeFailure(bytes, DecodeStatus::InvalidTeam);
    bytes = valid;
    bytes[18 + 3] = 6;
    expectDecodeFailure(bytes, DecodeStatus::InvalidGoalkeeper);
    bytes = valid;
    bytes[18] = 0;
    bytes[13] = game_controller_protocol_v20::kKickingTeamNone;
    expectDecodeFailure(bytes, DecodeStatus::InvalidTeam);
}

void testLegacyBoundariesAndValidation()
{
    const std::vector<std::uint8_t> valid = makeLegacyPacket();
    Packet packet{};
    assert(game_controller::decodePacket(valid.data(), valid.size(), packet) == DecodeStatus::Ok);
    auto specialState = valid;
    specialState[9] = STATE_DANIEL;
    assert(game_controller::decodePacket(specialState.data(), specialState.size(), packet) == DecodeStatus::Ok);
    assert(packet.state == STATE_DANIEL);
    assert(game_controller::legacyStateForRos(packet.state) == STATE_INITIAL);
    assert(game_controller::legacyStateRequiresStop(packet.state));
    auto boundary = valid;
    boundary[7] = static_cast<std::uint8_t>(game_controller::kLegacyMaxPlayers);
    assert(game_controller::decodePacket(boundary.data(), boundary.size(), packet) == DecodeStatus::Ok);
    assert(packet.playersPerTeam == game_controller::kLegacyMaxPlayers);
    assert(packet.teams[0].players[10].available);

    auto bytes = valid;
    bytes.resize(valid.size() - 1);
    expectDecodeFailure(bytes, DecodeStatus::InvalidLength);
    bytes = valid;
    bytes.push_back(0);
    expectDecodeFailure(bytes, DecodeStatus::InvalidLength);
    bytes = valid;
    bytes[4] = 15;
    expectDecodeFailure(bytes, DecodeStatus::UnsupportedVersion);
    bytes = valid;
    bytes[7] = 0;
    expectDecodeFailure(bytes, DecodeStatus::InvalidCount);
    bytes = valid;
    bytes[7] = 12;
    expectDecodeFailure(bytes, DecodeStatus::InvalidCount);
    bytes = valid;
    bytes[9] = 5;
    expectDecodeFailure(bytes, DecodeStatus::InvalidState);
    bytes = valid;
    bytes[10] = 2;
    expectDecodeFailure(bytes, DecodeStatus::InvalidPhase);
    bytes = valid;
    bytes[12] = 10;
    expectDecodeFailure(bytes, DecodeStatus::InvalidPhase);
    bytes = valid;
    bytes[24 + 266 + 5] = 2;
    expectDecodeFailure(bytes, DecodeStatus::InvalidGoalkeeper);
    bytes = valid;
    bytes[24 + 266 + 5] = 1;
    expectDecodeFailure(bytes, DecodeStatus::InvalidGoalkeeper);
    bytes = valid;
    bytes[24 + 266 + 6 + 5] = 0;
    bytes[24 + 266 + 5 * 6 + 5] = 1;
    expectDecodeFailure(bytes, DecodeStatus::InvalidGoalkeeper);
    bytes = valid;
    bytes[24] = 0;
    bytes[11] = DROPBALL;
    expectDecodeFailure(bytes, DecodeStatus::InvalidTeam);
    bytes = valid;
    bytes[24 + game_controller::kLegacyTeamSize] = 13;
    expectDecodeFailure(bytes, DecodeStatus::InvalidTeam);
}

} // namespace

int main()
{
    testOfficialDecodeAndSignedTimes();
    testLegacyDecode();
    testOfficialBoundariesAndValidation();
    testLegacyBoundariesAndValidation();
    std::puts("ALL GAME CONTROLLER PARSER TESTS PASSED");
    return 0;
}
