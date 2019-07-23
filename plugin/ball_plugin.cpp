#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include "message/input/gazebo/Ball.pb.h"
#include "nuclear_network.h"

namespace gazebo {
class NUbotsBallPlugin : public ModelPlugin {

public:
    // Inherit constructors
    using ModelPlugin::ModelPlugin;

    /**
     * The load function is called by Gazebo when the plugin is inserted into simulation
     * @param _model A pointer to the model that this plugin is attached to
     * @param _sdf   A pointer to the plugin's SDF element
     */
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

        // Output a debug line
        gzdbg << "Attaching a ball plugin to model [" << _model->GetName() << "]" << std::endl;

        // Store the model pointer for convenience
        model = _model;

        // Last update is when we load - 10ms
        last_update = std::chrono::steady_clock::now() - std::chrono::microseconds(10);

        // When we get an update event, publish the ball position and velocity
        update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&NUbotsBallPlugin::update_ball, this));
    }

private:
    void update_ball() {
        auto now = std::chrono::steady_clock::now();
        if (now - last_update > std::chrono::milliseconds(10)) {
            last_update += std::chrono::milliseconds(10);

            // Make our message
            auto msg = std::make_unique<message::input::gazebo::Ball>();
            auto p   = model->WorldPose().Pos();
            msg->mutable_rbww()->set_x(p.X());
            msg->mutable_rbww()->set_y(p.Y());
            msg->mutable_rbww()->set_z(p.Z());

            auto v = model->WorldLinearVel();
            msg->mutable_vbww()->set_x(v.X());
            msg->mutable_vbww()->set_y(v.Y());
            msg->mutable_vbww()->set_z(v.Z());

            // Emit the message using the NUClear network as an unreliable packet
            // get_reactor().emit<NUClear::dsl::word::emit::Network>(msg);
        }
    }

    // Pointer to the model
    physics::ModelPtr model;

    // Holds the callback from gazebo
    event::ConnectionPtr update_connection;

    // The last time we sent a packet so we can rate limit to about 100hz
    std::chrono::steady_clock::time_point last_update;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin
GZ_REGISTER_MODEL_PLUGIN(NUbotsBallPlugin)

}  // namespace gazebo
