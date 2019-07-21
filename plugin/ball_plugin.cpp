#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

namespace gazebo {
class NUbotsBallPlugin : public ModelPlugin {
public:
    NUbotsBallPlugin() {}

    /**
     * The load function is called by Gazebo when the plugin is inserted into simulation
     * @param _model A pointer to the model that this plugin is attached to
     * @param _sdf   A pointer to the plugin's SDF element
     */
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

        // Just output a message for now
        std::cerr << "\nThe NUbots ball plugin is attached to model [" << _model->GetName() << "]\n";

        // Store the model pointer for convenience
        this->model = _model;

        // Set up the update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&NUbotsBallPlugin::OnUpdate, this));

        // Set up the communcations with NUClear
        // setenv("IGN_PARTITION", "NubotsIgus", 1);
        // setenv("IGN_IP", "127.0.0.1", 1);
        static const int g_msgPort           = 11319;
        static std::string pUuid             = ignition::transport::Uuid().ToString();
        static std::string nUuid             = ignition::transport::Uuid().ToString();
        const static std::string topicStatus = "NubotsBallStatus";
        static std::string hostAddr          = ignition::transport::determineHost();
        static std::string ctrlAddr          = ignition::transport::determineHost();

        // Set up transport node for joint status
        // This will be ADVERTISED to the Status topic
        ignition::transport::NodeOptions ballStatusNodeOpts;
        ballStatusNodeOpts.SetPartition("Ball");
        ballStatusNodeOpts.SetNameSpace("nubots");
        ballStatus = new ignition::transport::Node(ballStatusNodeOpts);

        ignition::transport::AdvertiseMessageOptions AdMsgOpts;
        AdMsgOpts.SetMsgsPerSec(90u);
        discoveryNode = new ignition::transport::MsgDiscovery(pUuid, g_msgPort);
        msgPublisher  = new ignition::transport::MessagePublisher(
            topicStatus, hostAddr, ctrlAddr, pUuid, nUuid, "ADVERTISE", AdMsgOpts);

        // Set up a callback function for the discovery service
        std::function<void(const ignition::transport::MessagePublisher& _publisher)> onDiscoveryCb(
            [this](const ignition::transport::MessagePublisher& _publisher) -> void {
                std::cerr << "Discovered a Message Publisher!" << std::endl;
                std::cerr << _publisher << std::endl;
            });

        // Set up a callback function for the discovery service
        std::function<void(const ignition::transport::MessagePublisher& _publisher)> onDisconnectionCb(
            [this](const ignition::transport::MessagePublisher& _publisher) -> void {
                std::cerr << "Disconnected from a Message Publisher!" << std::endl;
                std::cerr << _publisher << std::endl;
            });

        discoveryNode->ConnectionsCb(onDiscoveryCb);
        discoveryNode->DisconnectionsCb(onDisconnectionCb);

        // Start the discovery service
        discoveryNode->Start();

        if (!discoveryNode->Advertise(*msgPublisher))
            std::cerr << "Failed to advertise the discovery node!" << std::endl;

        ignition::transport::AdvertiseMessageOptions opts;
        opts.SetMsgsPerSec(90u);
        // ADVERTISE the status node
        pub = ballStatus->Advertise<ignition::msgs::StringMsg>(topicStatus, opts);
    }

    void OnUpdate() {
        if (!pub.Publish(GetBallStatus())) std::cerr << "Error publishing to topic [topicStatus]" << std::endl;
    }

private:
    const ignition::msgs::StringMsg GetBallStatus() {
        ignition::msgs::StringMsg ballStatus;
        std::string string = "";
        string += "simulation1\n";

        double x, y, z;
        ignition::math::Pose3d pose;
        pose                         = this->model->WorldPose();
        ignition::math::Vector3d pos = pose.Pos();
        x                            = pos.X();  // x coordinate
        y                            = pos.Y();  // y coordinate
        z                            = pos.Z();  // z coordinate

        string +=
            std::to_string((float) x) + "\n" + std::to_string((float) y) + "\n" + std::to_string((float) z) + "\n";

        ignition::math::Vector3d vel = this->model->WorldLinearVel();

        x = vel.X();  // x coordinate
        y = vel.Y();  // y coordinate
        z = vel.Z();  // z coordinate

        string += std::to_string((float) x) + "\n" + std::to_string((float) y) + "\n" + std::to_string((float) z);

        ballStatus.set_data(string);
        return ballStatus;
    }

    /// \brief Pointer to the model
    physics::ModelPtr model;

    /// \brief Nodes used for sending ball information
    ignition::transport::Node* ballStatus;

    /// \brief Publisher used for sending ball information
    ignition::transport::Node::Publisher pub;

    /// \brief A subscriber to a named topic.
    ignition::transport::MsgDiscovery* discoveryNode;

    ignition::transport::MessagePublisher* msgPublisher;

    event::ConnectionPtr updateConnection;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin
GZ_REGISTER_MODEL_PLUGIN(NUbotsBallPlugin)

}  // namespace gazebo
