#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <iostream>
#include <string>
#include <array>
#include <asio.hpp>
#include <chrono>
#include <system_error>
#include <cstdio>
#include <sys/select.h>
#include <unistd.h>
#include <thread>

#define NUMBER_OF_SENSOR_THUMB 6
#define NUMBER_OF_SENSOR_FINGER 5
using asio::ip::udp;

inline bool check_for_user_input()
{
    fd_set read_fds;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);

    int ready = select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout);
    return ready > 0;
}

class UdpCommunicator {
public:
    UdpCommunicator(unsigned short port);
    void init();

    void send(const std::array<double, 16>& data, const udp::endpoint& endpoint);

    std::array<double, NUMBER_OF_SENSOR_THUMB +2*NUMBER_OF_SENSOR_FINGER> receive(udp::endpoint& sender_endpoint);

    void close();

    int getSocketFd();

private:
    asio::io_context io_context_;
    udp::endpoint endpoint_;
    udp::socket socket_;
};

#endif // UDP_RECEIVER_H
