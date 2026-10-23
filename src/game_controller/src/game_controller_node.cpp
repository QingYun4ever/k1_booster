#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include "game_controller_node.h"

namespace
{
constexpr int kReceivePollTimeoutMs = 100;
constexpr std::uint8_t kNoDropInTeam = game_controller_protocol_v20::kKickingTeamNone;
constexpr std::uint16_t kNoDropInTime = 0xffff;

void copyLegacyRobot(
    const game_controller::RobotInfo &source,
    game_controller_interface::msg::RobotInfo &target)
{
    target.penalty = source.penalty;
    target.secs_till_unpenalised = source.secsTillUnpenalised;
    target.number_of_warnings = source.numberOfWarnings;
    target.yellow_card_count = source.yellowCardCount;
    target.red_card_count = source.redCardCount;
    target.cautions = source.cautions;
    target.goal_keeper = source.goalKeeper != 0;
    target.available = source.available;
}

void copyCommonTeam(
    const game_controller::TeamInfo &source,
    game_controller_interface::msg::TeamInfo &target)
{
    target.team_number = source.teamNumber;
    target.field_player_colour = source.fieldPlayerColour;
    target.goalkeeper_colour = source.goalkeeperColour;
    target.goalkeeper = source.goalkeeper;
    target.score = source.score;
    target.penalty_shot = source.penaltyShot;
    target.single_shots = source.singleShots;
    target.message_budget = source.messageBudget;
}
} // namespace

GameControllerNode::GameControllerNode(std::string name) : rclcpp::Node(name)
{
    declare_parameter<int>("port", 3838);
    declare_parameter<bool>("enable_ip_white_list", false);
    declare_parameter<std::vector<std::string>>("ip_white_list", std::vector<std::string>{});

    get_parameter("port", _port);
    RCLCPP_INFO(get_logger(), "[get_parameter] port: %d", _port);
    get_parameter("enable_ip_white_list", _enable_ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] enable_ip_white_list: %d", _enable_ip_white_list);
    get_parameter("ip_white_list", _ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] ip_white_list(len=%zu)", _ip_white_list.size());
    for (std::size_t i = 0; i < _ip_white_list.size(); ++i)
    {
        RCLCPP_INFO(get_logger(), "[get_parameter]     --[%zu]: %s", i, _ip_white_list[i].c_str());
    }

    _publisher = create_publisher<game_controller_interface::msg::GameControlData>(
        "/robocup/game_controller", 10);
}

GameControllerNode::~GameControllerNode()
{
    _stopping.store(true, std::memory_order_release);
    if (_thread.joinable())
    {
        _thread.join();
    }
    if (_socket >= 0)
    {
        close(_socket);
        _socket = -1;
    }
}

void GameControllerNode::init()
{
    if (_socket >= 0 || _thread.joinable())
    {
        throw std::logic_error("GameControllerNode::init called more than once");
    }

    if (_port < 1 || _port > 65535) throw std::invalid_argument("GameController UDP port must be 1..65535");
    _socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(get_logger(), "socket failed: %s", strerror(errno));
        throw std::runtime_error(strerror(errno));
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(_port));
    if (bind(_socket, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0)
    {
        const int bindError = errno;
        RCLCPP_ERROR(get_logger(), "bind failed: %s (port=%d)", strerror(bindError), _port);
        close(_socket);
        _socket = -1;
        throw std::runtime_error(strerror(bindError));
    }

    _stopping.store(false, std::memory_order_release);
    RCLCPP_INFO(get_logger(), "Listening for UDP broadcast on 0.0.0.0:%d", _port);
    try
    {
        _thread = std::thread(&GameControllerNode::spin, this);
    }
    catch (...)
    {
        close(_socket);
        _socket = -1;
        throw;
    }
}

void GameControllerNode::spin()
{
    std::array<std::uint8_t, game_controller::kMaxPacketSize + 1> buffer{};
    game_controller::Packet data{};
    game_controller_interface::msg::GameControlData message;
    while (!_stopping.load(std::memory_order_acquire) && rclcpp::ok())
    {
        pollfd descriptor{};
        descriptor.fd = _socket;
        descriptor.events = POLLIN;
        const int pollResult = poll(&descriptor, 1, kReceivePollTimeoutMs);
        if (pollResult == 0)
        {
            continue;
        }
        if (pollResult < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            if (!_stopping.load(std::memory_order_acquire))
            {
                RCLCPP_ERROR(get_logger(), "poll failed: %s", strerror(errno));
            }
            break;
        }
        if ((descriptor.revents & POLLIN) == 0)
        {
            if (!_stopping.load(std::memory_order_acquire))
            {
                RCLCPP_ERROR(get_logger(), "UDP socket poll error: revents=0x%x", descriptor.revents);
            }
            break;
        }

        sockaddr_in remoteAddress{};
        socklen_t remoteAddressLength = sizeof(remoteAddress);
        const ssize_t received = recvfrom(
            _socket,
            buffer.data(),
            buffer.size(),
            0,
            reinterpret_cast<sockaddr *>(&remoteAddress),
            &remoteAddressLength);
        if (received < 0)
        {
            if (errno == EINTR || _stopping.load(std::memory_order_acquire))
            {
                continue;
            }
            RCLCPP_ERROR(get_logger(), "receiving UDP message failed: %s", strerror(errno));
            continue;
        }

        char remoteIp[INET_ADDRSTRLEN]{};
        if (inet_ntop(AF_INET, &remoteAddress.sin_addr, remoteIp, sizeof(remoteIp)) == nullptr)
        {
            RCLCPP_WARN(get_logger(), "could not format remote UDP address");
            continue;
        }
        if (!check_ip_white_list(remoteIp))
        {
            RCLCPP_INFO(get_logger(), "received packet from %s, but not in ip white list, ignore it", remoteIp);
            continue;
        }
        if (static_cast<std::size_t>(received) > game_controller::kMaxPacketSize)
        {
            RCLCPP_INFO(
                get_logger(), "packet from %s exceeds maximum size: %zd", remoteIp, received);
            continue;
        }

        const game_controller::DecodeStatus status = game_controller::decodePacket(
            buffer.data(), static_cast<std::size_t>(received), data);
        if (status != game_controller::DecodeStatus::Ok)
        {
            RCLCPP_INFO(
                get_logger(), "packet from %s rejected by GameController decoder: status=%u, length=%zd",
                remoteIp, static_cast<unsigned int>(status), received);
            continue;
        }

        handle_packet(data, message);
        _publisher->publish(message);
        RCLCPP_INFO(
            get_logger(), "handled GameController packet from %s, version=%u, packet_number=%u",
            remoteIp, static_cast<unsigned int>(data.version), static_cast<unsigned int>(data.packetNumber));
    }
}

bool GameControllerNode::check_ip_white_list(const std::string &ip) const
{
    if (!_enable_ip_white_list)
    {
        return true;
    }
    for (const std::string &allowedIp : _ip_white_list)
    {
        if (ip == allowedIp)
        {
            return true;
        }
    }
    return false;
}

void GameControllerNode::handle_packet(
    const game_controller::Packet &data,
    game_controller_interface::msg::GameControlData &message) const
{
    constexpr char header[] = "RGme";
    for (std::size_t i = 0; i < sizeof(message.header); ++i)
    {
        message.header[i] = header[i];
    }
    message.version = data.version;
    message.packet_number = data.packetNumber;
    message.players_per_team = data.playersPerTeam;
    message.state = data.state;
    message.first_half = data.firstHalf;
    message.kick_off_team = data.kickingTeam;
    message.secs_remaining = data.secsRemaining;
    message.secondary_time = data.secondaryTime;

    if (data.legacy)
    {
        message.state = game_controller::legacyStateForRos(data.state);
        message.stopped = game_controller::legacyStateRequiresStop(data.state);
        message.game_phase = 0;
        message.set_play = 0;
        message.secondary_state = data.secondaryState;
        message.drop_in_team = data.dropInTeam;
        message.drop_in_time = data.dropInTime;
        for (std::size_t i = 0; i < message.secondary_state_info.size(); ++i)
        {
            message.secondary_state_info[i] = data.secondaryStateInfo[i];
        }

        for (std::size_t teamIndex = 0; teamIndex < data.teams.size(); ++teamIndex)
        {
            const game_controller::TeamInfo &sourceTeam = data.teams[teamIndex];
            game_controller_interface::msg::TeamInfo &targetTeam = message.teams[teamIndex];
            copyCommonTeam(sourceTeam, targetTeam);
            targetTeam.coach_sequence = sourceTeam.coachSequence;
            targetTeam.coach_message.assign(sourceTeam.coachMessage.begin(), sourceTeam.coachMessage.end());
            copyLegacyRobot(sourceTeam.coach, targetTeam.coach);
            targetTeam.players.resize(game_controller::kLegacyMaxPlayers);
            for (std::size_t playerIndex = 0; playerIndex < game_controller::kLegacyMaxPlayers; ++playerIndex)
            {
                copyLegacyRobot(sourceTeam.players[playerIndex], targetTeam.players[playerIndex]);
            }
        }
        return;
    }

    message.game_type = data.competitionType;
    message.stopped = data.stopped != 0;
    message.game_phase = data.gamePhase;
    message.set_play = data.setPlay;
    message.secondary_state = game_controller_protocol_v20::toLegacySecondaryState(
        data.gamePhase, data.setPlay);
    message.secondary_state_info[0] = data.kickingTeam;
    message.secondary_state_info[1] = data.stopped == 0 ? 1 : 0;
    message.secondary_state_info[2] = 0;
    message.secondary_state_info[3] = 0;
    message.drop_in_team = kNoDropInTeam;
    message.drop_in_time = kNoDropInTime;

    for (std::size_t teamIndex = 0; teamIndex < data.teams.size(); ++teamIndex)
    {
        const game_controller::TeamInfo &sourceTeam = data.teams[teamIndex];
        game_controller_interface::msg::TeamInfo &targetTeam = message.teams[teamIndex];
        copyCommonTeam(sourceTeam, targetTeam);
        targetTeam.coach_sequence = 0;
        targetTeam.coach_message.clear();
        targetTeam.coach.penalty = 0;
        targetTeam.coach.secs_till_unpenalised = 0;
        targetTeam.coach.number_of_warnings = 0;
        targetTeam.coach.yellow_card_count = 0;
        targetTeam.coach.red_card_count = 0;
        targetTeam.coach.cautions = 0;
        targetTeam.coach.goal_keeper = false;
        targetTeam.coach.available = false;

        targetTeam.players.resize(game_controller::kOfficialMaxPlayers);
        for (std::size_t playerIndex = 0; playerIndex < game_controller::kOfficialMaxPlayers; ++playerIndex)
        {
            const game_controller::RobotInfo &sourcePlayer = sourceTeam.players[playerIndex];
            game_controller_interface::msg::RobotInfo &targetPlayer = targetTeam.players[playerIndex];
            targetPlayer.penalty = sourcePlayer.penalty;
            targetPlayer.secs_till_unpenalised = sourcePlayer.secsTillUnpenalised;
            targetPlayer.number_of_warnings = 0;
            targetPlayer.yellow_card_count = sourcePlayer.cautions;
            targetPlayer.red_card_count = sourcePlayer.penalty == game_controller_protocol_v20::kPenaltySentOff ? 1 : 0;
            targetPlayer.cautions = sourcePlayer.cautions;
            targetPlayer.goal_keeper = sourceTeam.goalkeeper == playerIndex + 1;
            targetPlayer.available = sourcePlayer.available;
        }
    }
}
