#include "nuclear_network.h"

#include <gazebo/gazebo.hh>

class NetworkReactor;

std::unique_ptr<NUClear::PowerPlant> powerplant = nullptr;
NetworkReactor* reactor                         = nullptr;
std::unique_ptr<std::thread> exec               = nullptr;
std::mutex init_mutex;

class NetworkReactor : public NUClear::Reactor {
public:
    NetworkReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        ::reactor = this;

        // Print out when the NUClear is starting up
        on<Startup>().then([this] {
            gzmsg << "NUClear is starting up" << std::endl;

            auto net_config              = std::make_unique<NUClear::message::NetworkConfiguration>();
            net_config->name             = "gazebo";
            net_config->announce_address = "239.226.152.162";
            net_config->announce_port    = 7447;
            emit<Scope::DIRECT>(net_config);
        });

        on<Trigger<NUClear::message::NetworkJoin>>().then([this](const NUClear::message::NetworkJoin& event) {
            char c[255];
            std::memset(c, 0, sizeof(c));
            std::string addr;
            int port;

            switch (event.address.sock.sa_family) {
                case AF_INET:
                    addr = inet_ntop(event.address.sock.sa_family, &event.address.ipv4.sin_addr, c, sizeof(c));
                    port = ntohs(event.address.ipv4.sin_port);
                    break;

                case AF_INET6:
                    addr = inet_ntop(event.address.sock.sa_family, &event.address.ipv6.sin6_addr, c, sizeof(c));
                    port = ntohs(event.address.ipv6.sin6_port);
                    break;
            }

            gzmsg << "NUClearNet: Connected to " << event.name << " on " << addr + ":" + std::to_string(port)
                  << std::endl;
        });

        on<Trigger<NUClear::message::NetworkLeave>>().then([this](const NUClear::message::NetworkLeave& event) {
            char c[255];
            std::memset(c, 0, sizeof(c));
            std::string addr;
            int port;

            switch (event.address.sock.sa_family) {
                case AF_INET:
                    addr = inet_ntop(event.address.sock.sa_family, &event.address.ipv4.sin_addr, c, sizeof(c));
                    port = ntohs(event.address.ipv4.sin_port);
                    break;

                case AF_INET6:
                    addr = inet_ntop(event.address.sock.sa_family, &event.address.ipv6.sin6_addr, c, sizeof(c));
                    port = ntohs(event.address.ipv6.sin6_port);
                    break;
            }

            gzmsg << "NUClearNet: Disconnected from " << event.name << " on " << addr + ":" + std::to_string(port)
                  << std::endl;
        });
    }
};

NUClear::Reactor& get_reactor() {
    // If we don't have a reactor yet, make one
    if (!powerplant) {
        std::lock_guard<std::mutex> lock(init_mutex);
        // Double check that we weren't duped by lock contention!
        if (!powerplant) {
            powerplant = std::make_unique<NUClear::PowerPlant>();
            powerplant->install<NetworkReactor>();

            // Start up the NUClear in another thread
            exec = std::make_unique<std::thread>([] { powerplant->start(); });
        }
    }
    return *reactor;
}
