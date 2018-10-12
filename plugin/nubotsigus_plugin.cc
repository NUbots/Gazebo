#ifndef _NUBOTSIGUS_PLUGIN_HH_
#define _NUBOTSIGUS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <nubots_common.hh>

using namespace ignition;
using namespace transport;

namespace gazebo
{
	class NUbotsIgusPlugin : public ModelPlugin
	{
	public:
		/// \brief Constructor
		NUbotsIgusPlugin() {}

		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to
		/// \param[in] _sdf A pointer to the plugin's SDF element
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// Safety check to see if the SDF file is attached correctly
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "Invalid joint count, NUbots Igus plugin not loaded" << std::endl;
				return;
			}

			// Just output a message for now
			std::cerr << "\nThe NUbots igus plugin is attached to model [" <<
				_model->GetName() << "]\n";

			// Store the model pointer for convenience
			this->model = _model;

			this->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::get_sensor("imu_sensor"));

			// Set up the update event
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&NUbotsIgusPlugin::OnUpdate, this));
			
			// Get the joint pointers
			this->joints = _model->GetJoints();

			// Set up the communcations with NUClear
			//setenv("IGN_PARTITION", "NubotsIgus", 1);
			//setenv("IGN_IP", "127.0.0.1", 1);
			static const int g_msgPort = 11319;
			static std::string pUuid = ignition::transport::Uuid().ToString();
			static std::string nUuid = ignition::transport::Uuid().ToString();
			const static std::string topicCtrl = "NubotsIgusCtrl";
			const static std::string topicStatus = "NubotsIgusStatus";
			static std::string hostAddr = ignition::transport::determineHost();
			static std::string ctrlAddr = ignition::transport::determineHost();

			// Set up transport node for joint control
			// This will be SUBSCRIBED to the Ctrl topic
			ignition::transport::NodeOptions jointCtrlNodeOpts;
			jointCtrlNodeOpts.SetPartition("Igus");
			jointCtrlNodeOpts.SetNameSpace(nubots_ign_namespace);
			jointCtrl = new ignition::transport::Node(jointCtrlNodeOpts);

			// Set up transport node for joint status
			// This will be ADVERTISED to the Status topic
			ignition::transport::NodeOptions jointStatusNodeOpts;
			jointStatusNodeOpts.SetPartition("Igus");
			jointStatusNodeOpts.SetNameSpace(nubots_ign_namespace);
			jointStatus = new ignition::transport::Node(jointStatusNodeOpts);

			ignition::transport::AdvertiseMessageOptions AdMsgOpts;
			AdMsgOpts.SetMsgsPerSec(90u);
			discoveryNode = new MsgDiscovery(pUuid, g_msgPort);
			msgPublisher = new ignition::transport::MessagePublisher(topicStatus, hostAddr, ctrlAddr, pUuid,
				nUuid, "ADVERTISE", AdMsgOpts);

			std::function<void(const ignition::msgs::StringMsg &_msg)> JointCtrlCb(
				[this](const ignition::msgs::StringMsg &_msg) -> void
				{
					std::stringstream ss(_msg.data());
					std::string line;
					int commandPresent;
					float target, gain, timeTar;

					std::cerr << ss.str() << "\n%%%%%%%%%%%%%%%%%%%%%" << std::endl;
					//std::cerr << _msg.data() << std::endl;
					
					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(17, (double)(target - 1.5708), (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(6, (double)(target - 1.5708), (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(18, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(7, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(19, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(8, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(11, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(0, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(12, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(1, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(13, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(2, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(14, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(3, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(15, (double)target, (double)gain, (double)timeTar);
						//std::cerr << "R_ANKLE_PITCH tar: " << target << ", gain: " << gain << std::endl;
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(4, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(16, (double)target, (double)gain, (double)timeTar);
						//std::cerr << "R_ANKLE_ROLL tar: " << target << ", gain: " << gain << std::endl;
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(5, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(9, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}

					std::getline(ss, line);
					commandPresent = std::stoi(line);
					if (commandPresent == 1)
					{
						std::getline(ss, line);
						target = std::stof(line);
						std::getline(ss, line);
						gain = std::stof(line);
						std::getline(ss, line);
						timeTar = std::stof(line);
						this->JointCommand(10, (double)target, (double)gain, (double)timeTar);
					}
					else 
					{
						std::getline(ss, line);
						std::getline(ss, line);
						std::getline(ss, line);
					}
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

			if (!discoveryNode->Discover(topicCtrl))
				std::cout << "discovery failed..." << std::endl;

			if (!jointCtrl->Subscribe<ignition::msgs::StringMsg>(topicCtrl, JointCtrlCb))
				std::cerr << "Error subscribing to joint commands messages at [" << topicCtrl << "]" << std::endl;

			ignition::transport::AdvertiseMessageOptions opts;
			opts.SetMsgsPerSec(90u);
			// ADVERTISE the status node
			pub = jointStatus->Advertise<ignition::msgs::StringMsg>(topicStatus, opts);

			// Setup a P-controller, with a gain of 4.
			this->pid = common::PID(40.0, 0.0, 0.0);

			// Apply the P-controller to the joint.
			for (int i = 0; i < this->joints.size(); i++)
			{
				this->model->GetJointController()->SetPositionPID(
					this->joints[i]->GetScopedName(), this->pid);

				// This will set the initial positions of the robot

				this->model->GetJointController()->SetPositionTarget(
					this->joints[i]->GetScopedName(), initialPositions[i]);
				
				this->joints[i]->SetPosition(0, initialPositions[i]);
				
				this->joints[i]->SetStopDissipation(0, 0.0);
				this->joints[i]->SetStiffness(0, 3.25);
				//this->joints[i]->SetStopStiffness (0, 0.125);
				//this->joints[i]->Set
				//this->joints[i]->SetStiffnessDamping(0, 4.0, 0.125, 0.0);
				//this->joints[i]->Set
				this->joints[i]->SetEffortLimit(0, 15.0);
			}
		}

		void OnUpdate()
		{
			if (!pub.Publish(GetRobotStatus()))
				std::cerr << "Error publishing to topic [topicStatus]" << std::endl;
		}
		
	private:
		/// \brief Set the command values for a joint
		/// \param[in] _id
		/// \param[in] _tarPos
		/// \param[in] _gain
		void JointCommand(const int &_id, const double &_tarPos, const double &_gain, const double &_tarTime)
		{
			this->pid = common::PID(_gain * 4.0);
			// Set the joint's gain by applying the
			// P-controller to the joints for positions.
			if (_id == 5 || _id == 16) {
				this->model->GetJointController()->SetPositionPID(
					this->joints[_id]->GetScopedName(), common::PID(_gain*0.7));
			}
			else {
				this->model->GetJointController()->SetPositionPID(
					this->joints[_id]->GetScopedName(), this->pid);
			}
			// Set the joint's target position.
			this->model->GetJointController()->SetPositionTarget(
				this->joints[_id]->GetScopedName(), _tarPos);

			/*if (std::abs(_tarTime) > 0.01)
			{
				this->model->GetJointController()->SetVelocityPID(
					this->joints[_id]->GetScopedName(), common::PID(1.0));
				this->model->GetJointController()->SetVelocityTarget(
					this->joints[_id]->GetScopedName(), _tarTime);
			}*/
			if (_tarTime > 0.0)
				this->joints[_id]->SetVelocityLimit(0, _tarTime);
			else 
				this->joints[_id]->SetVelocityLimit(0, 3.75);

			if (_id == 5 || _id == 16) { // ankle roll
				//this->joints[_id]->SetVelocityLimit(0, 0.012);
				//this->joints[_id]->SetEffortLimit(0, 40);
			}
			//this->joints[_id]->SetStiffnessDamping(0, 2.0, 0.8, _tarPos);
				
			std::cerr << "ID: " << _id << ", vel: " << _tarTime << std::endl;
			//this->model->GetJointController()->Update();
			
		}

		/// \brief Pointer to the model
		physics::ModelPtr model;

		/// \brief Pointer to the imu sensor
		sensors::ImuSensorPtr imuSensor;

		/// \brief Pointer to the joints
		std::vector<physics::JointPtr> joints;

		/// \brief A PID controller for the joints
		common::PID pid;

		//double gainFactor = 1.0;

		/// \brief Nodes used for controlling the joints
		ignition::transport::Node* jointCtrl;

		/// \brief Nodes used for sending joint information
		ignition::transport::Node* jointStatus;

		/// \brief Publisher used for sending joint information
		ignition::transport::Node::Publisher pub;

		/// \brief A subscriber to a named topic.
		ignition::transport::MsgDiscovery* discoveryNode;

		ignition::transport::MessagePublisher* msgPublisher;

		event::ConnectionPtr updateConnection;

		const ignition::msgs::StringMsg GetRobotStatus()
		{
			ignition::msgs::StringMsg jointStatus;
			std::string string = "";
			// need present position and speed
			// cast to float

			string += nubots_sim_name + "\n";
			// R_SHOULDER_PITCH
			string += std::to_string((float)this->joints[17]->GetVelocity(0)) + "\n";
			string += std::to_string((float)(this->joints[17]->Position() + 1.5708)) + "\n";
			// L_SHOULDER_PITCH
			string += std::to_string((float)this->joints[6]->GetVelocity(0)) + "\n";
			string += std::to_string((float)(this->joints[6]->Position() + 1.5708)) + "\n";
			// R_SHOULDER_ROLL
			string += std::to_string((float)this->joints[18]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[18]->Position()) + "\n";
			// L_SHOULDER_ROLL
			string += std::to_string((float)this->joints[7]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[7]->Position()) + "\n";
			// R_ELBOW
			string += std::to_string((float)this->joints[19]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[19]->Position()) + "\n";
			// L_ELBOW
			string += std::to_string((float)this->joints[8]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[8]->Position()) + "\n";
			// R_HIP_YAW
			string += std::to_string((float)this->joints[11]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[11]->Position()) + "\n";
			// L_HIP_YAW
			string += std::to_string((float)this->joints[0]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[0]->Position()) + "\n";
			// R_HIP_ROLL
			string += std::to_string((float)this->joints[12]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[12]->Position()) + "\n";
			// L_HIP_ROLL
			string += std::to_string((float)this->joints[1]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[1]->Position()) + "\n";
			// R_HIP_PITCH
			string += std::to_string((float)this->joints[13]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[13]->Position()) + "\n";
			// L_HIP_PITCH
			string += std::to_string((float)this->joints[2]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[2]->Position()) + "\n";
			// R_KNEE
			string += std::to_string((float)this->joints[14]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[14]->Position()) + "\n";
			// L_KNEE
			string += std::to_string((float)this->joints[3]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[3]->Position()) + "\n";
			// R_ANKLE_PITCH
			string += std::to_string((float)this->joints[15]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[15]->Position()) + "\n";
			// L_ANKLE_PITCH
			string += std::to_string((float)this->joints[4]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[4]->Position()) + "\n";
			// R_ANKLE_ROLL
			string += std::to_string((float)this->joints[16]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[16]->Position()) + "\n";
			// L_ANKLE_ROLL
			string += std::to_string((float)this->joints[5]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[5]->Position()) + "\n";
			// HEAD_YAW
			string += std::to_string((float)this->joints[9]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[9]->Position()) + "\n";
			// HEAD_PITCH
			string += std::to_string((float)this->joints[10]->GetVelocity(0)) + "\n";
			string += std::to_string((float)this->joints[10]->Position()) + "\n";
			//std::cerr << "HeadPitch vel: " << (float)this->joints[10]->GetVelocity(0)
			//	<< ", HeadPitch pos: " << (float)this->joints[10]->Position() << std::endl;

			ignition::math::Vector3d angularVel = this->imuSensor->AngularVelocity();
			ignition::math::Vector3d linearAcc = this->imuSensor->LinearAcceleration();
			
			string += std::to_string(angularVel.X()) + "\n"
							+ std::to_string(angularVel.Y()) + "\n"
							+ std::to_string(angularVel.Z()) + "\n"
							+ std::to_string(linearAcc.X() ) + "\n"
							+ std::to_string(linearAcc.Y() ) + "\n"
							+ std::to_string(linearAcc.Z() ) + "\n";

			double x,y,z;
			ignition::math::Pose3d pose;
			pose = this->model->WorldPose();
			ignition::math::Vector3d pos = pose.Pos();
			x = pos.X(); // x coordinate
			y = pos.Y(); // y coordinate
			z = pos.Z(); // z coordinate
			
			string += std::to_string((float)x) + "\n" +
								std::to_string((float)y) + "\n" +
								std::to_string((float)z);

			jointStatus.set_data(string);
			return jointStatus;
		}

		const double initialPositions[20] = {
			0.02924,
			0.063,
			-0.207,
			0.25614,
			-0.24,
			-0.07,
			0.4,
			0.123,
			-2.443,
			0.0,
			0.0,
			-0.02924,
			-0.063,
			-0.207,
			0.25614,
			-0.24,
			0.07,
			0.4,
			-0.123,
			-2.443
		}; 
		
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin
	GZ_REGISTER_MODEL_PLUGIN(NUbotsIgusPlugin)
}
#endif
