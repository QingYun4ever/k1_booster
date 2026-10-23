#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "RoboCupGameControlData.h"
#include "team_communication_msg.h"

class Brain;

class BrainCommunication
{
public:
    explicit BrainCommunication(Brain *argBrain);
    ~BrainCommunication();

    void initCommunication();

    // Called on Brain's main thread: apply queued teammate state and publish
    // immutable snapshots consumed by the socket workers.
    void update();
    void setGameControllerProtocolVersion(uint16_t version);

private:
    using SteadyClock = std::chrono::steady_clock;
    static constexpr size_t COMMUNICATION_QUEUE_CAPACITY = 64;

    struct TeammateInfo {
        uint32_t ip = 0;
        int playerId = 0;
        SteadyClock::time_point lastUpdate;
    };

    struct QueuedCommunicationPacket {
        TeamCommunicationMsg msg;
        SteadyClock::time_point receivedAt;
    };

    struct GcSnapshot {
        bool valid = false;
        uint8_t playerNum = 0;
        uint8_t teamNum = 0;
        uint8_t fallen = 0;
        float pose[3] = {0.f, 0.f, 0.f};
        float ball[2] = {0.f, 0.f};
        float ballAgeSeconds = -1.f;
        SteadyClock::time_point capturedAt;
    };

    struct TxDiagnostics {
        bool hasSent = false;
        uint32_t communicationId = 0;
        uint32_t ip = 0;
        SteadyClock::time_point sentAt;
    };

    Brain *brain;
    int _team_id = 0;
    int _player_id = 0;
    bool _valid_communication_identity = false;

    void initGameControllerUnicast();
    void unicastToGameController();
    void clearupGameControllerUnicast();
    std::thread _gamecontrol_unicast_thread;
    std::atomic<bool> _unicast_gamecontrol_flag{false};
    int _gc_send_socket = -1;
    sockaddr_in _gcsaddr{};
    std::atomic<uint16_t> _gc_protocol_version{12};
    GcSnapshot _gc_snapshot;
    std::mutex _gc_snapshot_mutex;
    static constexpr int BROADCAST_GAME_CONTROL_INTERVAL_MS = 1000;

    std::atomic<uint32_t> _discovery_msg_id{0};
    std::atomic<bool> _broadcast_discovery_flag{false};
    int _discovery_send_socket = -1;
    int _discovery_udp_port = 0;
    sockaddr_in _saddr{};
    std::thread _discovery_broadcast_thread;
    static constexpr int BROADCAST_DISCOVERY_INTERVAL_MS = 1000;
    void initDiscoveryBroadcast();
    void clearupDiscoveryBroadcast();
    void broadcastDiscovery();

    std::atomic<bool> _receive_discovery_flag{false};
    std::thread _discovery_recv_thread;
    int _discovery_recv_socket = -1;
    void initDiscoveryReceiver();
    void clearupDiscoveryReceiver();
    void spinDiscoveryReceiver();

    std::map<uint32_t, TeammateInfo> _teammate_addresses;
    std::mutex _teammate_addresses_mutex;
    std::array<bool, HL_MAX_NUM_PLAYERS> _received_discovery_id_valid{};
    std::array<uint32_t, HL_MAX_NUM_PLAYERS> _last_received_discovery_id{};
    static constexpr int TEAMMATE_TIMEOUT_MS = 20 * 1000;
    void cleanupExpiredTeammates();

    void initCommunicationUnicast();
    void clearupCommunicationUnicast();
    void unicastCommunication();
    std::atomic<bool> _unicast_communication_flag{false};
    std::thread _unicast_thread;
    int _unicast_socket = -1;
    int _unicast_udp_port = 0;
    sockaddr_in _unicast_saddr{};
    static constexpr int OUTBOUND_SNAPSHOT_STALE_MS = 2200;
    static constexpr int UNICAST_INTERVAL_MS = 100;

    void initCommunicationReceiver();
    void clearupCommunicationReceiver();
    void spinCommunicationReceiver();
    std::atomic<bool> _receive_communication_flag{false};
    std::thread _communication_recv_thread;
    int _communication_recv_socket = -1;
    int _communication_recv_port = 0;

    TeamCommunicationMsg _outbound_snapshot;
    bool _outbound_snapshot_valid = false;
    SteadyClock::time_point _outbound_snapshot_time;
    std::mutex _outbound_snapshot_mutex;

    std::array<QueuedCommunicationPacket, COMMUNICATION_QUEUE_CAPACITY> _communication_queue{};
    size_t _communication_queue_head = 0;
    size_t _communication_queue_size = 0;
    std::mutex _communication_queue_mutex;
    std::array<bool, HL_MAX_NUM_PLAYERS> _received_communication_id_valid{};
    std::array<uint32_t, HL_MAX_NUM_PLAYERS> _last_received_communication_id{};
    std::array<bool, HL_MAX_NUM_PLAYERS> _received_ball_observation_id_valid{};
    std::array<uint32_t, HL_MAX_NUM_PLAYERS> _last_received_ball_observation_id{};

    uint32_t _next_communication_id = 0;
    bool _has_local_ball_observation_time = false;
    int64_t _last_local_ball_observation_time_ns = 0;
    uint32_t _local_ball_observation_id = 0;
    bool _local_ball_observation_location_known = false;
    float _local_ball_observation_field_x = 0.f;
    float _local_ball_observation_field_y = 0.f;

    std::mutex _tx_diagnostics_mutex;
    TxDiagnostics _tx_diagnostics;
    uint32_t _last_published_send_id = 0;
    uint32_t _last_published_discovery_id = 0;
};
