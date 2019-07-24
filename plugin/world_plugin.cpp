#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "message/input/gazebo/Command.pb.h"
#include "message/input/gazebo/Simulation.pb.h"
#include "nuclear_network.h"

namespace gazebo {
class NUbotsWorldPlugin : public WorldPlugin {
public:
    // Inherit constructors
    using WorldPlugin::WorldPlugin;

    template <typename T>
    using Network = NUClear::dsl::word::Network<T>;

    // Make sure to unbind the reaction handle on destruction
    virtual ~NUbotsWorldPlugin() {
        handle.unbind();
    }

    /**
     * The load function is called by Gazebo when the plugin is inserted into simulation
     * @param _model A pointer to the model that this plugin is attached to
     * @param _sdf   A pointer to the plugin's SDF element
     */
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

        // Last update is when we load
        last_update = std::chrono::steady_clock::now() - std::chrono::milliseconds(10);

        // Just output a message for now
        gzdbg << "Attaching a NUbots world plugin to [" << _world->Name() << "]" << std::endl;

        // Configure the NUClear network
        auto net_config              = std::make_unique<NUClear::message::NetworkConfiguration>();
        net_config->name             = _sdf->Get<std::string>("nuclearnet_name", "gazebo").first;
        net_config->announce_address = _sdf->Get<std::string>("nuclearnet_address", "239.226.152.162").first;
        net_config->announce_port    = _sdf->Get<int>("nuclearnet_port", 7447).first;
        get_reactor().emit<NUClear::dsl::word::emit::Direct>(net_config);

        // Store the model pointer for convenience
        this->world = _world;

        // When we get an update event, publish the ball position and velocity
        update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&NUbotsWorldPlugin::update_world, this));

        handle = get_reactor().on<Network<message::input::gazebo::Command>>().then(
            [this](const message::input::gazebo::Command& c) {
                // Store this command ready for the next update as we don't want to mess with gazebos threading
                std::lock_guard<std::mutex> lock(command_mutex);
                command_queue.push_back(c);
            });
    }


private:
    void update_world() {
        // Run all our commands in the command queue and then clear it
        {
            std::lock_guard<std::mutex> lock(command_mutex);
            for (const auto& c : command_queue) {
                command(c);
            }
            command_queue.resize(0);
        }

        // Only send packets at 100Hz
        auto now = std::chrono::steady_clock::now();
        if (now - last_update > std::chrono::milliseconds(10)) {
            last_update += std::chrono::milliseconds(10);
            // Make our message
            auto msg = std::make_unique<message::input::gazebo::Simulation>();
            msg->set_sim_time(this->world->SimTime().Double());
            msg->set_real_time(this->world->RealTime().Double());

            // Emit the message to everyone (broadcast) using the NUClear network as an unreliable packet
            get_reactor().emit<NUClear::dsl::word::emit::Network>(msg);
        }
    }

    void command(const message::input::gazebo::Command& cmd) {
        switch (cmd.type()) {
            case message::input::gazebo::Command::Type::Command_Type_RESET: world->Reset(); break;
            case message::input::gazebo::Command::Type::Command_Type_RESET_TIME: world->ResetTime(); break;
            default: break;
        }
    }

    // Pointer to the world state object
    physics::WorldPtr world;

    // Holds the callback from gazebo
    event::ConnectionPtr update_connection;

    // The last time we sent a packet so we can rate limit to about 100hz
    std::chrono::steady_clock::time_point last_update;

    // A reaction handle so we can unbind when we are destructed
    NUClear::threading::ReactionHandle handle;

    // A queue of commands to be executed on the next simulation frame
    std::mutex command_mutex;
    std::vector<message::input::gazebo::Command> command_queue;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(NUbotsWorldPlugin)

}  // namespace gazebo
