#ifndef _NUBOTSWORLD_PLUGIN_HH_
#define _NUBOTSWORLD_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>

#include "nubots_common.h"

using namespace ignition;
using namespace transport;

namespace gazebo
{
	class NUbotsWorldPlugin : public WorldPlugin
	{
	public: 
		/// \brief Constructor
		NUbotsWorldPlugin() {}
		
		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _world A pointer to the world that this plugin is
		/// attached to
		/// \param[in] _sdf A pointer to the plugin's SDF element
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
		{
			
			// Just output a message for now
			std::cerr << "\nThe NUbots world plugin is attached to world [" <<
				_world->Name() << "]\n";

			// Store the model pointer for convenience
			this->world = _world;

			// Set up the update event
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&NUbotsWorldPlugin::OnUpdate, this));

			// Set up the communcations with NUClear
			//setenv("IGN_PARTITION", "NubotsIgus", 1);
			//setenv("IGN_IP", "127.0.0.1", 1);
			static const int g_msgPort = 11319;
			static std::string pUuid = ignition::transport::Uuid().ToString();
			static std::string nUuid = ignition::transport::Uuid().ToString();
			const static std::string topicWorldCtrl = "NubotsWorldCtrl";
			const static std::string topicWorldStatus = "NubotsWorldStatus";
			static std::string hostAddr = ignition::transport::determineHost();
			static std::string ctrlAddr = ignition::transport::determineHost();

			// Set up transport node for world control
			// This will be SUBSCRIBED to the Ctrl topic
			ignition::transport::NodeOptions worldCtrlNodeOpts;
			worldCtrlNodeOpts.SetPartition("World");
			worldCtrlNodeOpts.SetNameSpace(nubots_ign_namespace);
			worldCtrl = new ignition::transport::Node(worldCtrlNodeOpts);

			// Set up transport node for World status
			// This will be ADVERTISED to the Status topic
			ignition::transport::NodeOptions worldStatusNodeOpts;
			worldStatusNodeOpts.SetPartition("World");
			worldStatusNodeOpts.SetNameSpace(nubots_ign_namespace);
			worldStatus = new ignition::transport::Node(worldStatusNodeOpts);

			ignition::transport::AdvertiseMessageOptions AdMsgOpts;
			discoveryNode = new MsgDiscovery(pUuid, g_msgPort);
			msgPublisher = new ignition::transport::MessagePublisher(topicWorldStatus, hostAddr, ctrlAddr, pUuid,
				nUuid, "ADVERTISE", AdMsgOpts);

			std::function<void(const ignition::msgs::StringMsg &_msg)> WorldCtrlCb(
				[this](const ignition::msgs::StringMsg &_msg) -> void
				{
					std::stringstream ss(_msg.data());
					std::string line;

					std::getline(ss, line);

					if (line == "RESET")
						this->world->Reset();
					else if (line == "RESETTIME")
						this->world->ResetTime();
				}
			);

			// Set up a callback function for the discovery service
			std::function<void(const ignition::transport::MessagePublisher &_publisher)> onDiscoveryCb(
				[this](const ignition::transport::MessagePublisher &_publisher) -> void
				{
					std::cerr << "Discovered a Message Publisher!" << std::endl;
					std::cerr << _publisher << std::endl;
				});

			// Set up a callback function for the discovery service
			std::function<void(const ignition::transport::MessagePublisher &_publisher)> onDisconnectionCb(
				[this](const ignition::transport::MessagePublisher &_publisher) -> void
				{
					std::cerr << "Disconnected from a Message Publisher!" << std::endl;
					std::cerr << _publisher << std::endl;
				});

			discoveryNode->ConnectionsCb(onDiscoveryCb);
			discoveryNode->DisconnectionsCb(onDisconnectionCb);

			// Start the discovery service
			discoveryNode->Start();

			if (!discoveryNode->Advertise(*msgPublisher))
				std::cerr << "Failed to advertise the discovery node!" << std::endl;

			if (!discoveryNode->Discover(topicWorldCtrl))
				std::cout << "discovery failed..." << std::endl;

			if (!worldCtrl->Subscribe<ignition::msgs::StringMsg>(topicWorldCtrl, WorldCtrlCb))
				std::cerr << "Error subscribing to joint commands messages at [" << topicWorldCtrl << "]" << std::endl;

			ignition::transport::AdvertiseMessageOptions opts;
			opts.SetMsgsPerSec(100u);
			// ADVERTISE the status node
			worldPub = worldStatus->Advertise<ignition::msgs::StringMsg>(topicWorldStatus, opts);
		}

		void OnUpdate()
		{
			ignition::msgs::StringMsg worldStatus;
			worldStatus.set_data(std::to_string(this->world->SimTime().Double()) + "\n"
				+ std::to_string(this->world->RealTime().Double()));
			if (!worldPub.Publish(worldStatus))
				std::cerr << "Error publishing to world status topic" << std::endl;
		}

	private:
		/// \brief Pointer to the world
		physics::WorldPtr world;

		/// \brief Nodes used for controlling the world
		ignition::transport::Node* worldCtrl;

		/// \brief Nodes used for sending world information
		ignition::transport::Node* worldStatus;

		/// \brief Publisher used for sending joint information
		ignition::transport::Node::Publisher worldPub;

		/// \brief A subscriber to a named topic.
		ignition::transport::MsgDiscovery* discoveryNode;

		ignition::transport::MessagePublisher* msgPublisher;

		event::ConnectionPtr updateConnection;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(NUbotsWorldPlugin)
}

#endif
