#pragma once

#include "ICanBus.hpp"
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace CANLib {

class SocketCanBus : public ICanBus {
public:
    SocketCanBus();
    ~SocketCanBus() override;

    bool open(const std::string& interface_name, ReceiveCallback callback) override;
    void close() override;
    bool send(const CanFrame& frame) override;

private:
    static constexpr size_t kMaxTxQueueDepth = 256;

    int socketFd_;
    ReceiveCallback receiveCallback_;
    std::thread receiveThread_;
    std::thread txThread_;
    std::atomic<bool> running_;
    std::string interfaceName_;

    std::queue<CanFrame> txQueue_;
    std::mutex txMutex_;
    std::condition_variable txCv_;

    void receiveLoop_();
    void txLoop_();
};

} // namespace CANLib