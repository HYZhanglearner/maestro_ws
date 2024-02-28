#include "UDP_receiver.h"

UdpCommunicator::UdpCommunicator(unsigned short port)
    : endpoint_(udp::v4(), port),
      socket_(io_context_, endpoint_) {
    // Do nothing
}

void UdpCommunicator::init(){

    // This function is kept for consistency, but it's not needed since
    // the constructor initializes the socket and endpoint.

}

void UdpCommunicator::send(const std::array<double, 16> &data, const udp::endpoint& endpoint){
    socket_.send_to(asio::buffer(data), endpoint);
}

std::array<double, NUMBER_OF_SENSOR_THUMB +2*NUMBER_OF_SENSOR_FINGER> UdpCommunicator::receive(udp::endpoint& sender_endpoint) {
    std::array<double, NUMBER_OF_SENSOR_THUMB +2*NUMBER_OF_SENSOR_FINGER> data;
    std::error_code error;

    socket_.non_blocking(true);  // Set the socket to non-blocking mode

    int attempts = 0;
    int max_attempts = 1000; // adjust this for desired timeout
    while (attempts < max_attempts) {
        // Try to receive data
        socket_.receive_from(asio::buffer(data), sender_endpoint, 0, error);

        if (error) {
            if (error == asio::error::would_block || error == asio::error::try_again) {
                // No data was available, but otherwise the receive was successful
                // We'll wait a bit and then try again
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
                attempts++;
                continue;
            } else {
                // Some other error occurred
                throw std::system_error(error);
            }
        }

        // If we got here, we received some data and can return it
        return data;
    }

    // If we got here, we didn't receive any data within the timeout period
    throw std::system_error(asio::error::timed_out);
}


void UdpCommunicator::close(){
    socket_.close();
}

int UdpCommunicator::getSocketFd() {
    return socket_.native_handle();
}
