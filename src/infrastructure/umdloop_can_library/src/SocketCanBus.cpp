#include "umdloop_can_library/SocketCanBus.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>
#include <thread>

namespace CANLib {

namespace {
    void mapCanFrameToSocketCan(const CanFrame& frame, ::can_frame& can_frame) {
        std::memset(&can_frame, 0, sizeof(can_frame));
        can_frame.can_id = frame.id;
        can_frame.can_dlc = frame.dlc;

        if (frame.is_extended) {
            can_frame.can_id |= CAN_EFF_FLAG;
        }
        if (frame.is_rtr) {
            can_frame.can_id |= CAN_RTR_FLAG;
        }

        std::copy(frame.data.begin(), frame.data.begin() + frame.dlc, can_frame.data);
    }

    void mapSocketCanToCanFrame(const ::can_frame& can_frame, CanFrame& frame) {
        frame.id = can_frame.can_id & CAN_ERR_MASK;
        frame.dlc = std::min(can_frame.can_dlc, static_cast<__u8>(8));
        frame.is_extended = (can_frame.can_id & CAN_EFF_FLAG) != 0;
        frame.is_rtr = (can_frame.can_id & CAN_RTR_FLAG) != 0;
        frame.data.fill(0);
        std::copy(can_frame.data, can_frame.data + frame.dlc, frame.data.begin());
    }
}

SocketCanBus::SocketCanBus() : socketFd_(-1), running_(false) {}

SocketCanBus::~SocketCanBus() {
    close();
}

bool SocketCanBus::open(const std::string& interface_name, ReceiveCallback callback) {
    if (running_) return false;

    receiveCallback_ = callback;
    interfaceName_ = interface_name;

    // Create socket
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketFd_ < 0) {
        std::cerr << "Failed to create SocketCAN socket: " << strerror(errno) << std::endl;
        return false;
    }

    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socketFd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get interface index for " << interface_name
                  << ": " << strerror(errno) << std::endl;
        ::close(socketFd_);
        socketFd_ = -1;
        return false;
    }

    // Bind socket to CAN interface
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketFd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind socket to " << interface_name
                  << ": " << strerror(errno) << std::endl;
        ::close(socketFd_);
        socketFd_ = -1;
        return false;
    }

    // Disable loopback (do not receive our own transmitted frames)
    int loopback = 0;
    setsockopt(socketFd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // Disable CAN FD — we only use classical CAN frames
    int canfd_off = 0;
    setsockopt(socketFd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_off, sizeof(canfd_off));

    // Increase the socket send buffer to 1 MB (kernel doubles this, giving ~2 MB effective)
    int sndbuf = 1048576;
    setsockopt(socketFd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    running_ = true;
    receiveThread_ = std::thread(&SocketCanBus::receiveLoop_, this);
    txThread_ = std::thread(&SocketCanBus::txLoop_, this);
    return true;
}

void SocketCanBus::close() {
    if (running_) {
        running_ = false;
        txCv_.notify_one();
        if (txThread_.joinable()) {
            txThread_.join();
        }
        if (receiveThread_.joinable()) {
            receiveThread_.join();
        }
        if (socketFd_ >= 0) {
            ::close(socketFd_);
            socketFd_ = -1;
        }
    }
}

bool SocketCanBus::send(const CanFrame& frame) {
    std::lock_guard<std::mutex> lock(txMutex_);
    if (txQueue_.size() >= kMaxTxQueueDepth) {
        std::cerr << "CAN TX queue full, dropping frame ID: 0x" << std::hex << frame.id << std::dec << std::endl;
        return false;
    }
    txQueue_.push(frame);
    txCv_.notify_one();
    return true;
}

void SocketCanBus::txLoop_() {
    while (running_) {
        std::unique_lock<std::mutex> lock(txMutex_);
        txCv_.wait(lock, [this] {
            return !txQueue_.empty() || !running_;
        });

        while (!txQueue_.empty()) {
            CanFrame frame = txQueue_.front();
            txQueue_.pop();
            lock.unlock();

            ::can_frame cf;
            mapCanFrameToSocketCan(frame, cf);

            int retries = 0;
            while (write(socketFd_, &cf, sizeof(cf)) != sizeof(cf)) {
                if ((errno == ENOBUFS || errno == EAGAIN) && running_ && retries++ < 10) {
                    std::this_thread::sleep_for(std::chrono::microseconds(500));
                } else {
                    std::cerr << "CAN TX failed - ID: 0x" << std::hex << frame.id
                              << std::dec << " Error: " << strerror(errno) << std::endl;
                    break;
                }

            }
            std::this_thread::sleep_for(std::chrono::microseconds(250));


            lock.lock();
        }
    }
}

void SocketCanBus::receiveLoop_() {
    ::can_frame can_frame;

    while (running_) {
        fd_set readfds;
        struct timeval timeout;

        FD_ZERO(&readfds);
        FD_SET(socketFd_, &readfds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100ms timeout

        int result = select(socketFd_ + 1, &readfds, NULL, NULL, &timeout);

        if (result < 0) {
            if (errno != EINTR) {
                std::cerr << "SocketCAN select error: " << strerror(errno) << std::endl;
                break;
            }
            continue;
        }

        if (result == 0) {
            continue;
        }

        if (FD_ISSET(socketFd_, &readfds)) {
            ssize_t nbytes = read(socketFd_, &can_frame, sizeof(can_frame));

            if (nbytes == sizeof(can_frame) && receiveCallback_) {
                CanFrame frame;
                mapSocketCanToCanFrame(can_frame, frame);
                receiveCallback_(frame);
            } else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "SocketCAN read error: " << strerror(errno) << std::endl;
                break;
            }
        }
    }
}

} // namespace CANLib