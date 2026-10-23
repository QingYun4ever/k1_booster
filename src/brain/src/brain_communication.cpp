#include "brain.h"
#include "brain_communication.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <cerrno>
#include <sys/time.h>

namespace
{
constexpr int MAX_TEAMMATE_PLAYER_ID = HL_MAX_NUM_PLAYERS;
constexpr size_t MAX_COMMUNICATION_DATAGRAM_SIZE = 256;


float toFiniteFloat(double value)
{
    if (!std::isfinite(value) || value > std::numeric_limits<float>::max() ||
        value < -std::numeric_limits<float>::max()) {
        return 0.f;
    }
    return static_cast<float>(value);
}

float toNonnegativeFloat(double value)
{
    return std::isfinite(value) && value >= 0.0 ? toFiniteFloat(value) : 0.f;
}

bool setReceiveTimeout(int socketFd)
{
    timeval timeout{};
    timeout.tv_usec = 200000;
    return setsockopt(socketFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0;
}

void reportSocketError(const char *operation)
{
    std::cerr << operation << " failed: " << std::strerror(errno) << '\n';
}
}

BrainCommunication::BrainCommunication(Brain *argBrain) : brain(argBrain)
{
}

BrainCommunication::~BrainCommunication()
{
    clearupGameControllerUnicast();
    clearupDiscoveryBroadcast();
    clearupDiscoveryReceiver();
    clearupCommunicationUnicast();
    clearupCommunicationReceiver();
}

void BrainCommunication::initCommunication()
{
    _team_id = brain->config->teamId;
    _player_id = brain->config->playerId;
    _valid_communication_identity = _team_id > 0 && _team_id <= 255 &&
                                    _player_id >= 1 && _player_id <= MAX_TEAMMATE_PLAYER_ID;

    initGameControllerUnicast();
    if (!brain->config->enableCom) {
        std::cout << "Communication disabled.\n";
        return;
    }
    if (!_valid_communication_identity) {
        std::cerr << "Communication disabled: invalid team/player identity.\n";
        return;
    }

    std::cout << "Communication enabled.\n";
    _discovery_udp_port = 20000 + _team_id;
    _unicast_udp_port = 30000 + _team_id;
    _communication_recv_port = _unicast_udp_port;
    initDiscoveryBroadcast();
    initDiscoveryReceiver();
    initCommunicationUnicast();
    initCommunicationReceiver();
}

void BrainCommunication::setGameControllerProtocolVersion(uint16_t version)
{
    _gc_protocol_version.store(version, std::memory_order_relaxed);
}

void BrainCommunication::initGameControllerUnicast()
{
    _gc_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_gc_send_socket < 0) {
        reportSocketError("GameController socket");
        return;
    }

    try {
        const std::string gamecontrolIp = brain->get_parameter("game_control_ip").as_string();
        std::cout << "GameControl IP: " << gamecontrolIp << '\n';
        _gcsaddr.sin_family = AF_INET;
        _gcsaddr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);
        if (inet_pton(AF_INET, gamecontrolIp.c_str(), &_gcsaddr.sin_addr) != 1) {
            std::cerr << "Invalid GameController IP address: " << gamecontrolIp << '\n';
            close(_gc_send_socket);
            _gc_send_socket = -1;
            return;
        }
        _unicast_gamecontrol_flag.store(true, std::memory_order_relaxed);
        _gamecontrol_unicast_thread = std::thread([this]() { unicastToGameController(); });
    } catch (const std::exception &e) {
        std::cerr << "GameController communication initialization failed: " << e.what() << '\n';
        close(_gc_send_socket);
        _gc_send_socket = -1;
    }
}

void BrainCommunication::clearupGameControllerUnicast()
{
    _unicast_gamecontrol_flag.store(false, std::memory_order_relaxed);
    if (_gamecontrol_unicast_thread.joinable()) {
        _gamecontrol_unicast_thread.join();
    }
    if (_gc_send_socket >= 0) {
        close(_gc_send_socket);
        _gc_send_socket = -1;
    }
}

void BrainCommunication::unicastToGameController()
{
    while (_unicast_gamecontrol_flag.load(std::memory_order_relaxed)) {
        if (_valid_communication_identity && _gc_protocol_version.load(std::memory_order_relaxed) == 20) {
            GcSnapshot snapshot;
            {
                std::lock_guard<std::mutex> lock(_gc_snapshot_mutex);
                snapshot = _gc_snapshot;
            }
            if (snapshot.valid) {
                RoboCupGameControlReturnData packet;
                packet.playerNum = snapshot.playerNum;
                packet.teamNum = snapshot.teamNum;
                packet.fallen = snapshot.fallen;
                for (size_t i = 0; i < 3; ++i) packet.pose[i] = snapshot.pose[i];
                packet.ball[0] = snapshot.ball[0];
                packet.ball[1] = snapshot.ball[1];
                if (snapshot.ballAgeSeconds >= 0.f) {
                    const auto elapsed = std::chrono::duration<double>(SteadyClock::now() - snapshot.capturedAt).count();
                    packet.ballAge = static_cast<float>(snapshot.ballAgeSeconds + elapsed);
                } else {
                    packet.ballAge = -1.f;
                }
                const ssize_t sent = sendto(_gc_send_socket, &packet, sizeof(packet), 0,
                                            reinterpret_cast<const sockaddr *>(&_gcsaddr), sizeof(_gcsaddr));
                if (sent < 0) reportSocketError("GameController v20 return");
            }
        } else if (_valid_communication_identity) {
            HlRoboCupGameControlReturnData packet;
            packet.team = static_cast<uint8_t>(_team_id);
            packet.player = static_cast<uint8_t>(_player_id);
            packet.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
            const ssize_t sent = sendto(_gc_send_socket, &packet, sizeof(packet), 0,
                                        reinterpret_cast<const sockaddr *>(&_gcsaddr), sizeof(_gcsaddr));
            if (sent < 0) reportSocketError("GameController v12 return");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(BROADCAST_GAME_CONTROL_INTERVAL_MS));
    }
}

void BrainCommunication::initDiscoveryBroadcast()
{
    _discovery_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_discovery_send_socket < 0) {
        reportSocketError("Discovery broadcast socket");
        return;
    }
    int broadcast = 1;
    if (setsockopt(_discovery_send_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        reportSocketError("Discovery SO_BROADCAST");
        close(_discovery_send_socket);
        _discovery_send_socket = -1;
        return;
    }

    _saddr.sin_family = AF_INET;
    _saddr.sin_addr.s_addr = INADDR_BROADCAST;
    _saddr.sin_port = htons(_discovery_udp_port);
    _broadcast_discovery_flag.store(true, std::memory_order_relaxed);
    _discovery_broadcast_thread = std::thread([this]() { broadcastDiscovery(); });
}

void BrainCommunication::clearupDiscoveryBroadcast()
{
    _broadcast_discovery_flag.store(false, std::memory_order_relaxed);
    if (_discovery_broadcast_thread.joinable()) {
        _discovery_broadcast_thread.join();
    }
    if (_discovery_send_socket >= 0) {
        close(_discovery_send_socket);
        _discovery_send_socket = -1;
    }
}

void BrainCommunication::broadcastDiscovery()
{
    while (_broadcast_discovery_flag.load(std::memory_order_relaxed)) {
        TeamDiscoveryMsg msg;
        const uint32_t id = _discovery_msg_id.fetch_add(1, std::memory_order_relaxed) + 1;
        msg.communicationId = static_cast<int32_t>(id);
        msg.teamId = _team_id;
        msg.playerId = _player_id;
        const ssize_t sent = sendto(_discovery_send_socket, &msg, sizeof(msg), 0,
                                    reinterpret_cast<const sockaddr *>(&_saddr), sizeof(_saddr));
        if (sent < 0) reportSocketError("Discovery broadcast sendto");
        std::this_thread::sleep_for(std::chrono::milliseconds(BROADCAST_DISCOVERY_INTERVAL_MS));
    }
}

void BrainCommunication::initDiscoveryReceiver()
{
    _discovery_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_discovery_recv_socket < 0) {
        reportSocketError("Discovery receiver socket");
        return;
    }
    int reuse = 1;
    if (setsockopt(_discovery_recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        reportSocketError("Discovery SO_REUSEADDR");
        close(_discovery_recv_socket);
        _discovery_recv_socket = -1;
        return;
    }
    if (!setReceiveTimeout(_discovery_recv_socket)) {
        reportSocketError("Discovery receive timeout");
        close(_discovery_recv_socket);
        _discovery_recv_socket = -1;
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_discovery_udp_port);
    if (bind(_discovery_recv_socket, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0) {
        reportSocketError("Discovery bind");
        close(_discovery_recv_socket);
        _discovery_recv_socket = -1;
        return;
    }
    _receive_discovery_flag.store(true, std::memory_order_relaxed);
    _discovery_recv_thread = std::thread([this]() { spinDiscoveryReceiver(); });
}

void BrainCommunication::clearupDiscoveryReceiver()
{
    _receive_discovery_flag.store(false, std::memory_order_relaxed);
    if (_discovery_recv_thread.joinable()) _discovery_recv_thread.join();
    if (_discovery_recv_socket >= 0) {
        close(_discovery_recv_socket);
        _discovery_recv_socket = -1;
    }
}

void BrainCommunication::spinDiscoveryReceiver()
{
    std::array<uint8_t, sizeof(TeamDiscoveryMsg) + 1> bytes{};
    while (_receive_discovery_flag.load(std::memory_order_relaxed)) {
        sockaddr_in addr{};
        socklen_t addrLen = sizeof(addr);
        const ssize_t length = recvfrom(_discovery_recv_socket, bytes.data(), bytes.size(), 0,
                                        reinterpret_cast<sockaddr *>(&addr), &addrLen);
        if (length < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            if (_receive_discovery_flag.load(std::memory_order_relaxed)) reportSocketError("Discovery recvfrom");
            continue;
        }
        if (static_cast<size_t>(length) != sizeof(TeamDiscoveryMsg)) continue;

        TeamDiscoveryMsg msg;
        std::memcpy(&msg, bytes.data(), sizeof(msg));
        if (msg.validation != VALIDATION_DISCOVERY || msg.communicationId < 0 ||
            msg.teamId != _team_id || msg.playerId < 1 || msg.playerId > MAX_TEAMMATE_PLAYER_ID ||
            msg.playerId == _player_id) {
            continue;
        }
        const size_t playerIndex = static_cast<size_t>(msg.playerId - 1);
        const uint32_t sequence = static_cast<uint32_t>(msg.communicationId);
        const auto now = SteadyClock::now();
        std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
        if (_received_discovery_id_valid[playerIndex] &&
            !team_communication_wire::isNewerSequence(sequence, _last_received_discovery_id[playerIndex])) {
            continue;
        }
        _received_discovery_id_valid[playerIndex] = true;
        _last_received_discovery_id[playerIndex] = sequence;
        _teammate_addresses[addr.sin_addr.s_addr] = {addr.sin_addr.s_addr, msg.playerId, now};
    }
}

void BrainCommunication::cleanupExpiredTeammates()
{
    const auto now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
    std::array<bool, HL_MAX_NUM_PLAYERS> expiredPlayers{};
    for (auto it = _teammate_addresses.begin(); it != _teammate_addresses.end();) {
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second.lastUpdate).count();
        if (age > TEAMMATE_TIMEOUT_MS) {
            std::cout << "Teammate id " << it->second.playerId << " timed out\n";
            if (it->second.playerId >= 1 && it->second.playerId <= HL_MAX_NUM_PLAYERS) {
                expiredPlayers[static_cast<size_t>(it->second.playerId - 1)] = true;
            }
            it = _teammate_addresses.erase(it);
        } else {
            ++it;
        }
    }
    for (const auto &entry : _teammate_addresses) {
        if (entry.second.playerId >= 1 && entry.second.playerId <= HL_MAX_NUM_PLAYERS) {
            expiredPlayers[static_cast<size_t>(entry.second.playerId - 1)] = false;
        }
    }
    for (size_t i = 0; i < expiredPlayers.size(); ++i) {
        if (expiredPlayers[i]) {
            _received_discovery_id_valid[i] = false;
            _last_received_discovery_id[i] = 0;
        }
    }

}
void BrainCommunication::initCommunicationUnicast()
{
    _unicast_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_unicast_socket < 0) {
        reportSocketError("Teammate unicast socket");
        return;
    }
    _unicast_saddr.sin_family = AF_INET;
    _unicast_saddr.sin_port = htons(_unicast_udp_port);
    _unicast_communication_flag.store(true, std::memory_order_relaxed);
    _unicast_thread = std::thread([this]() { unicastCommunication(); });
}

void BrainCommunication::unicastCommunication()
{
    while (_unicast_communication_flag.load(std::memory_order_relaxed)) {
        cleanupExpiredTeammates();
        TeamCommunicationMsg msg;
        bool snapshotValid = false;
        SteadyClock::time_point snapshotTime;
        {
            std::lock_guard<std::mutex> lock(_outbound_snapshot_mutex);
            msg = _outbound_snapshot;
            snapshotValid = _outbound_snapshot_valid;
            snapshotTime = _outbound_snapshot_time;
        }

        if (snapshotValid) {
            ++_next_communication_id;
            if (_next_communication_id == 0) ++_next_communication_id;
            msg.communicationId = _next_communication_id;

            const auto snapshotAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(SteadyClock::now() - snapshotTime).count();
            if (snapshotAgeMs > OUTBOUND_SNAPSHOT_STALE_MS) {
                msg.isAlive = false;
                msg.isLead = false;
                msg.isInVisualKick = false;
            }
            if (msg.ballDetected) {
                const uint64_t ageMs = static_cast<uint64_t>(msg.ballObservationAgeMs) +
                                       static_cast<uint64_t>(std::max<int64_t>(snapshotAgeMs, 0));
                if (ageMs > team_communication_wire::MAX_BALL_AGE_MS) {
                    msg.ballDetected = false;
                    msg.ballLocationKnown = false;
                    msg.ballObservationAgeMs = team_communication_wire::NO_BALL_AGE_MS;
                    msg.ballConfidence = 0.f;
                    msg.ballRange = 0.f;
                    msg.ballX = 0.f;
                    msg.ballY = 0.f;
                } else {
                    msg.ballObservationAgeMs = static_cast<uint16_t>(ageMs);
                }
            }

            std::array<uint8_t, team_communication_wire::PACKET_SIZE> bytes{};
            size_t packetSize = 0;
            if (team_communication_wire::pack(msg, bytes.data(), bytes.size(), packetSize)) {
                std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
                for (const auto &entry : _teammate_addresses) {
                    _unicast_saddr.sin_addr.s_addr = entry.second.ip;
                    const ssize_t sent = sendto(_unicast_socket, bytes.data(), packetSize, 0,
                                                reinterpret_cast<const sockaddr *>(&_unicast_saddr), sizeof(_unicast_saddr));
                    if (sent < 0) {
                        reportSocketError("Teammate sendto");
                    } else {
                        std::lock_guard<std::mutex> txLock(_tx_diagnostics_mutex);
                        _tx_diagnostics.hasSent = true;
                        _tx_diagnostics.communicationId = msg.communicationId;
                        _tx_diagnostics.ip = entry.second.ip;
                        _tx_diagnostics.sentAt = SteadyClock::now();
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(UNICAST_INTERVAL_MS));
    }
}

void BrainCommunication::clearupCommunicationUnicast()
{
    _unicast_communication_flag.store(false, std::memory_order_relaxed);
    if (_unicast_thread.joinable()) _unicast_thread.join();
    if (_unicast_socket >= 0) {
        close(_unicast_socket);
        _unicast_socket = -1;
    }
}

void BrainCommunication::initCommunicationReceiver()
{
    _communication_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_communication_recv_socket < 0) {
        reportSocketError("Teammate receiver socket");
        return;
    }
    int reuse = 1;
    if (setsockopt(_communication_recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        reportSocketError("Teammate SO_REUSEADDR");
        close(_communication_recv_socket);
        _communication_recv_socket = -1;
        return;
    }
    if (!setReceiveTimeout(_communication_recv_socket)) {
        reportSocketError("Teammate receive timeout");
        close(_communication_recv_socket);
        _communication_recv_socket = -1;
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_communication_recv_port);
    if (bind(_communication_recv_socket, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0) {
        reportSocketError("Teammate bind");
        close(_communication_recv_socket);
        _communication_recv_socket = -1;
        return;
    }
    _receive_communication_flag.store(true, std::memory_order_relaxed);
    _communication_recv_thread = std::thread([this]() { spinCommunicationReceiver(); });
}

void BrainCommunication::spinCommunicationReceiver()
{
    std::array<uint8_t, MAX_COMMUNICATION_DATAGRAM_SIZE> bytes{};
    while (_receive_communication_flag.load(std::memory_order_relaxed)) {
        const ssize_t length = recvfrom(_communication_recv_socket, bytes.data(), bytes.size(), 0, nullptr, nullptr);
        if (length < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            if (_receive_communication_flag.load(std::memory_order_relaxed)) reportSocketError("Teammate recvfrom");
            continue;
        }

        TeamCommunicationMsg msg;
        if (!team_communication_wire::unpack(bytes.data(), static_cast<size_t>(length), msg) ||
            msg.teamId != _team_id || msg.playerId == _player_id) {
            continue;
        }
        QueuedCommunicationPacket packet;
        packet.msg = msg;
        packet.receivedAt = SteadyClock::now();
        std::lock_guard<std::mutex> lock(_communication_queue_mutex);
        if (_communication_queue_size == COMMUNICATION_QUEUE_CAPACITY) {
            continue;
        }
        const size_t tail = (_communication_queue_head + _communication_queue_size) % COMMUNICATION_QUEUE_CAPACITY;
        _communication_queue[tail] = packet;
        ++_communication_queue_size;
    }
}

void BrainCommunication::clearupCommunicationReceiver()
{
    _receive_communication_flag.store(false, std::memory_order_relaxed);
    if (_communication_recv_thread.joinable()) _communication_recv_thread.join();
    if (_communication_recv_socket >= 0) {
        close(_communication_recv_socket);
        _communication_recv_socket = -1;
    }
}

void BrainCommunication::update()
{
    if (brain == nullptr || brain->data == nullptr || brain->config == nullptr) return;

    const auto now = brain->get_clock()->now();
    const auto nowSteady = SteadyClock::now();
    const bool localBallFresh = brain->hasFreshLocalBall();
    const GameObject &localBall = brain->data->ball;
    const bool localBallLocationKnown = localBallFresh && brain->tree && brain->tree->getEntry<bool>("ball_location_known");

    if (localBallFresh) {
        const int64_t observationTimeNs = localBall.timePoint.nanoseconds();
        if (!_has_local_ball_observation_time || observationTimeNs != _last_local_ball_observation_time_ns) {
            ++_local_ball_observation_id;
            if (_local_ball_observation_id == 0) ++_local_ball_observation_id;
            _last_local_ball_observation_time_ns = observationTimeNs;
            _has_local_ball_observation_time = true;
            _local_ball_observation_location_known = localBallLocationKnown;
            _local_ball_observation_field_x = toFiniteFloat(localBall.posToField.x);
            _local_ball_observation_field_y = toFiniteFloat(localBall.posToField.y);
        }
    }

    TeamCommunicationMsg outbound;
    outbound.teamId = static_cast<uint16_t>(_team_id);
    outbound.playerId = static_cast<uint8_t>(_player_id);
    outbound.playerRole = brain->tree && brain->tree->getEntry<std::string>("player_role") == "goal_keeper" ? 2 : 1;
    outbound.isAlive = brain->data->tmImAlive;
    outbound.isInVisualKick = outbound.isAlive && brain->data->tmImInVisualKick;
    outbound.isLead = outbound.isAlive && brain->data->tmImLead;
    outbound.ballDetected = localBallFresh;
    outbound.ballLocationKnown = localBallFresh && _local_ball_observation_location_known;
    outbound.ballObservationAgeMs = team_communication_wire::NO_BALL_AGE_MS;
    outbound.ballObservationId = _local_ball_observation_id;
    if (localBallFresh) {
        const double ageSeconds = (now.nanoseconds() - localBall.timePoint.nanoseconds()) / 1e9;
        if (!team_communication_wire::encodeBallObservationAge(ageSeconds, outbound.ballObservationAgeMs)) {
            outbound.ballDetected = false;
            outbound.ballLocationKnown = false;
        }
    }
    if (!outbound.ballDetected) outbound.ballObservationAgeMs = team_communication_wire::NO_BALL_AGE_MS;
    outbound.ballConfidence = outbound.ballDetected ? toFiniteFloat(localBall.confidence) : 0.f;
    outbound.ballRange = outbound.ballDetected ? toNonnegativeFloat(localBall.range) : 0.f;
    outbound.cost = toNonnegativeFloat(brain->data->tmMyCost);
    outbound.ballX = outbound.ballDetected ? _local_ball_observation_field_x : 0.f;
    outbound.ballY = outbound.ballDetected ? _local_ball_observation_field_y : 0.f;
    outbound.robotX = toFiniteFloat(brain->data->robotPoseToField.x);
    outbound.robotY = toFiniteFloat(brain->data->robotPoseToField.y);
    outbound.robotTheta = toFiniteFloat(brain->data->robotPoseToField.theta);
    outbound.kickDir = toFiniteFloat(brain->data->kickDir);
    outbound.thetaRb = toFiniteFloat(brain->data->robotBallAngleToField);

    {
        std::lock_guard<std::mutex> lock(_outbound_snapshot_mutex);
        _outbound_snapshot = outbound;
        _outbound_snapshot_valid = _valid_communication_identity;
        _outbound_snapshot_time = nowSteady;
    }

    GcSnapshot gcSnapshot;
    gcSnapshot.valid = true;
    gcSnapshot.playerNum = static_cast<uint8_t>(_player_id);
    gcSnapshot.teamNum = static_cast<uint8_t>(_team_id);
    gcSnapshot.fallen = brain->data->recoveryState == RobotRecoveryState::IS_READY ? 0 : 1;
    gcSnapshot.pose[0] = toFiniteFloat(brain->data->robotPoseToField.x * 1000.0);
    gcSnapshot.pose[1] = toFiniteFloat(brain->data->robotPoseToField.y * 1000.0);
    gcSnapshot.pose[2] = toFiniteFloat(brain->data->robotPoseToField.theta);
    const bool hasLocalBallObservation = _has_local_ball_observation_time;
    gcSnapshot.ball[0] = hasLocalBallObservation ? toFiniteFloat(localBall.posToRobot.x * 1000.0) : 0.f;
    gcSnapshot.ball[1] = hasLocalBallObservation ? toFiniteFloat(localBall.posToRobot.y * 1000.0) : 0.f;
    if (hasLocalBallObservation) {
        const double ageSeconds = (now.nanoseconds() - localBall.timePoint.nanoseconds()) / 1e9;
        gcSnapshot.ballAgeSeconds = std::isfinite(ageSeconds) && ageSeconds >= 0.0 ? toFiniteFloat(ageSeconds) : -1.f;
    }
    gcSnapshot.capturedAt = nowSteady;
    {
        std::lock_guard<std::mutex> lock(_gc_snapshot_mutex);
        _gc_snapshot = gcSnapshot;
    }
    const int configuredPlayerCount = brain->data->gcReceived
        ? std::min(brain->config->numOfPlayers, brain->data->gcPlayersPerTeam)
        : brain->config->numOfPlayers;
    const size_t playerCount = static_cast<size_t>(std::clamp(configuredPlayerCount, 0, HL_MAX_NUM_PLAYERS));

    size_t incomingCount = 0;
    {
        std::lock_guard<std::mutex> lock(_communication_queue_mutex);
        incomingCount = _communication_queue_size;
    }
    for (size_t packetIndex = 0; packetIndex < incomingCount; ++packetIndex) {
        QueuedCommunicationPacket queued;
        {
            std::lock_guard<std::mutex> lock(_communication_queue_mutex);
            if (_communication_queue_size == 0) break;
            queued = _communication_queue[_communication_queue_head];
            _communication_queue_head = (_communication_queue_head + 1) % COMMUNICATION_QUEUE_CAPACITY;
            --_communication_queue_size;
        }
        const TeamCommunicationMsg &msg = queued.msg;
        if (msg.teamId != _team_id || msg.playerId < 1 || msg.playerId > HL_MAX_NUM_PLAYERS ||
            msg.playerId == _player_id) {
            continue;
        }
        const size_t playerIndex = static_cast<size_t>(msg.playerId - 1);
        if (playerIndex >= playerCount) continue;
        if (brain->data->penalty[playerIndex] == PENALTY_SUBSTITUTE) continue;

        TMStatus &status = brain->data->tmStatus[playerIndex];
        if (_received_communication_id_valid[playerIndex] &&
            (now - status.timeLastCom).nanoseconds() > static_cast<int64_t>(TEAMMATE_TIMEOUT_MS) * 1000000LL) {
            _received_communication_id_valid[playerIndex] = false;
            _last_received_communication_id[playerIndex] = 0;
            _received_ball_observation_id_valid[playerIndex] = false;
            _last_received_ball_observation_id[playerIndex] = 0;
            status.communicationId = 0;
            status.ballObservationId = 0;
            status.ballObservationTime = rclcpp::Time(0, 0, now.get_clock_type());
            status.ballDetected = false;
            status.ballLocationKnown = false;
            status.isAlive = false;
            status.isLead = false;
            status.isInVisualKick = false;
        }
        if (_received_communication_id_valid[playerIndex] &&
            !team_communication_wire::isNewerSequence(msg.communicationId, _last_received_communication_id[playerIndex])) {
            continue;
        }
        _received_communication_id_valid[playerIndex] = true;
        _last_received_communication_id[playerIndex] = msg.communicationId;

        const auto receivedAgo = std::chrono::duration<double>(nowSteady - queued.receivedAt).count();
        const auto receiptTime = now - rclcpp::Duration::from_seconds(std::max(receivedAgo, 0.0));
        status.role = msg.playerRole == 1 ? "striker" : (msg.playerRole == 2 ? "goal_keeper" : "unknown");
        status.isAlive = msg.isAlive &&
            (!brain->data->gcReceived || brain->data->penalty[playerIndex] == PENALTY_NONE);
        status.ballDetected = msg.ballDetected;
        if (!msg.ballDetected) status.ballLocationKnown = false;
        status.cost = msg.cost;
        status.isLead = status.isAlive && msg.isLead;
        status.isInVisualKick = status.isAlive && msg.isInVisualKick;
        status.robotPoseToField = {msg.robotX, msg.robotY, msg.robotTheta};
        status.kickDir = msg.kickDir;
        status.timeLastCom = receiptTime;
        status.communicationId = msg.communicationId;

        if (msg.ballDetected &&
            (!_received_ball_observation_id_valid[playerIndex] ||
             team_communication_wire::isNewerSequence(msg.ballObservationId, _last_received_ball_observation_id[playerIndex]))) {
            _received_ball_observation_id_valid[playerIndex] = true;
            _last_received_ball_observation_id[playerIndex] = msg.ballObservationId;
            status.ballObservationId = msg.ballObservationId;
            status.ballLocationKnown = msg.ballLocationKnown;
            status.ballConfidence = msg.ballConfidence;
            status.ballRange = msg.ballRange;
            status.ballPosToField = {msg.ballX, msg.ballY, 0.0};
            status.thetaRb = msg.thetaRb;
            double ballAgeSeconds = 0.0;
            if (team_communication_wire::decodeBallObservationAge(msg.ballObservationAgeMs, ballAgeSeconds)) {
                status.ballObservationTime = receiptTime - rclcpp::Duration::from_seconds(ballAgeSeconds);
            }
        }
    }

    TxDiagnostics tx;
    {
        std::lock_guard<std::mutex> lock(_tx_diagnostics_mutex);
        tx = _tx_diagnostics;
    }
    if (tx.hasSent && tx.communicationId != _last_published_send_id) {
        _last_published_send_id = tx.communicationId;
        brain->data->sendId = static_cast<int>(tx.communicationId);
        const double elapsed = std::chrono::duration<double>(nowSteady - tx.sentAt).count();
        brain->data->sendTime = now - rclcpp::Duration::from_seconds(std::max(elapsed, 0.0));
        char ip[INET_ADDRSTRLEN]{};
        in_addr address{};
        address.s_addr = tx.ip;
        if (inet_ntop(AF_INET, &address, ip, sizeof(ip)) != nullptr) brain->data->tmIP = ip;
    }

    const uint32_t discoveryId = _discovery_msg_id.load(std::memory_order_relaxed);
    if (discoveryId != 0 && discoveryId != _last_published_discovery_id) {
        _last_published_discovery_id = discoveryId;
        brain->data->discoveryMsgId = static_cast<int>(discoveryId);
        brain->data->discoveryMsgTime = now;
    }
}
