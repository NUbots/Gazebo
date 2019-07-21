#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include "message/input/RawSensors.pb.h"
#include "message/motion/ServoTarget.pb.h"
#include "nuclear_network.h"

namespace gazebo {
class NUbotsIgusPlugin : public ModelPlugin {
private:
    const double initial_positions[20] = {0.02924, 0.063,  -0.207, 0.25614, -0.24,    -0.07,  0.4,
                                          0.123,   -2.443, 0.0,    0.0,     -0.02924, -0.063, -0.207,
                                          0.25614, -0.24,  0.07,   0.4,     -0.123,   -2.443};

    R_SHOULDER_PITCH = 17;
    L_SHOULDER_PITCH = 6;
    R_SHOULDER_ROLL  = 18;
    L_SHOULDER_ROLL  = 7;
    R_ELBOW          = 19;
    L_ELBOW          = 8;
    R_HIP_YAW        = 11;
    L_HIP_YAW        = 0;
    R_HIP_ROLL       = 12;
    L_HIP_ROLL       = 1;
    R_HIP_PITCH      = 13;
    L_HIP_PITCH      = 2;
    R_KNEE           = 14;
    L_KNEE           = 3;
    R_ANKLE_PITCH    = 15;
    L_ANKLE_PITCH    = 4;
    R_ANKLE_ROLL     = 16;
    L_ANKLE_ROLL     = 5;
    HEAD_YAW         = 9;
    HEAD_PITCH       = 10;

    const double servo_id_to_joint[20] = {};
    const double joint_to_servo_id[20] = {};

public:
    // Inherit constructors
    using ModelPlugin::ModelPlugin;

    template <typename T>
    using Network = NUClear::dsl::word::Network<T>;

    /**
     * The load function is called by Gazebo when the plugin is inserted into simulation
     * @param _model A pointer to the model that this plugin is attached to
     * @param _sdf   A pointer to the plugin's SDF element
     */
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

        // Safety check to see if the SDF file is attached correctly
        if (_model->GetJointCount() == 0) {
            gzerr << "Invalid joint count, NUbots Igus plugin not loaded" << std::endl;
            return;
        }

        // Just output a message for now
        gzerr << "Attaching an iGus plugin to model [" << _model->GetName() << "]" << std::endl;

        // Store the model pointer for convenience
        this->model = _model;

        this->imu_sensor = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::get_sensor("imu_sensor"));

        // Get the joint pointers
        this->joints = _model->GetJoints();

        // Set up the update event
        this->update_connection =
            event::Events::ConnectWorldUpdateBegin(std::bind(&NUbotsIgusPlugin::update_robot, this));

        get_reactor().on<Network<message::motion::ServoTargets>>().then(
            [this](const message::motion::ServoTargets& targets) {
                // Store the incoming commands in the queue for the next simulation frame
                std::lock_guard<std::mutex> lock(command_mutex);
                for (const auto& c : targets.targets()) {
                    command_queue.push_back(c);
                }
            });

        std::function<void(const ignition::msgs::StringMsg& _msg)> JointCtrlCb(
            [this](const ignition::msgs::StringMsg& _msg) -> void {
                std::stringstream ss(_msg.data());
                std::string line;
                int commandPresent;
                float target, gain, timeTar;

                std::cerr << ss.str() << "\n%%%%%%%%%%%%%%%%%%%%%" << std::endl;
                // std::cerr << _msg.data() << std::endl;

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(17, (double) (target - 1.5708), (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(6, (double) (target - 1.5708), (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(18, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(7, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(19, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(8, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(11, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(0, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(12, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(1, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(13, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(2, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(14, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(3, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(15, (double) target, (double) gain, (double) timeTar);
                    // std::cerr << "R_ANKLE_PITCH tar: " << target << ", gain: " << gain << std::endl;
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(4, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(16, (double) target, (double) gain, (double) timeTar);
                    // std::cerr << "R_ANKLE_ROLL tar: " << target << ", gain: " << gain << std::endl;
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(5, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(9, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }

                std::getline(ss, line);
                commandPresent = std::stoi(line);
                if (commandPresent == 1) {
                    std::getline(ss, line);
                    target = std::stof(line);
                    std::getline(ss, line);
                    gain = std::stof(line);
                    std::getline(ss, line);
                    timeTar = std::stof(line);
                    this->JointCommand(10, (double) target, (double) gain, (double) timeTar);
                }
                else {
                    std::getline(ss, line);
                    std::getline(ss, line);
                    std::getline(ss, line);
                }
            });

        // Setup a P-controller, with a gain of 4.
        this->pid = common::PID(40.0, 0.0, 0.0);

        // Apply the P-controller to the joint.
        for (int i = 0; i < this->joints.size(); i++) {
            this->model->GetJointController()->SetPositionPID(this->joints[i]->GetScopedName(), this->pid);

            // This will set the initial positions of the robot
            this->model->GetJointController()->SetPositionTarget(this->joints[i]->GetScopedName(),
                                                                 initial_positions[i]);

            this->joints[i]->SetPosition(0, initial_positions[i]);

            this->joints[i]->SetStopDissipation(0, 0.0);
            this->joints[i]->SetStiffness(0, 3.25);
            // this->joints[i]->SetStopStiffness (0, 0.125);
            // this->joints[i]->Set
            // this->joints[i]->SetStiffnessDamping(0, 4.0, 0.125, 0.0);
            // this->joints[i]->Set
            this->joints[i]->SetEffortLimit(0, 20.0);

            this->joints[i]->SetStiffnessDamping(0, 3.25, 0.25, this->joints[i]->Position());
        }
    }

private:
    void apply_joint_command(const message::motion::ServoTarget& target) {

        this->pid = common::PID(target.gain() * 4.0);
        // Set the joint's gain by applying the
        // P-controller to the joints for positions.
        if (_id == 5 || _id == 16) {
            this->model->GetJointController()->SetPositionPID(this->joints[_id]->GetScopedName(),
                                                              common::PID(_gain * 0.7));
        }
        else {
            this->model->GetJointController()->SetPositionPID(this->joints[_id]->GetScopedName(), this->pid);
        }
        // Set the joint's target position.
        this->model->GetJointController()->SetPositionTarget(this->joints[_id]->GetScopedName(), _tarPos);

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

        if (_id == 5 || _id == 16) {  // ankle roll
                                      // this->joints[_id]->SetVelocityLimit(0, 0.012);
            // this->joints[_id]->SetEffortLimit(0, 40);
        }
        this->joints[_id]->SetStiffnessDamping(0, 3.25, 0.25, this->joints[_id]->Position());

        std::cerr << "ID: " << _id << ", vel: " << _tarTime << std::endl;
        // this->model->GetJointController()->Update();
    }

    void update_robot() {

        // Run all our commands in the command queue and then clear it
        {
            std::lock_guard<std::mutex> lock(command_mutex);
            for (const auto& c : command_queue) {
                apply_join_command(c);
            }
            command_queue.resize(0);
        }

        auto now = std::chrono::steady_clock::now();
        if (now - last_update > std::chrono::milliseconds(10)) {
            last_update += std::chrono::milliseconds(10);

            auto msg = std::make_unique<message::input::RawSensors>();
            for (int i = 0; i < 20; ++i) {
                joints[i]->GetVelocity(0);
                joints[i]->Position();
            }
        }
        // ignition::msgs::StringMsg jointStatus;
        // std::string string = "";
        // // need present position and speed
        // // cast to float

        // string += "simulation1\n";
        // // R_SHOULDER_PITCH
        // string += std::to_string((float) this->joints[17]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) (this->joints[17]->Position() + 1.5708)) + "\n";
        // // L_SHOULDER_PITCH
        // string += std::to_string((float) this->joints[6]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) (this->joints[6]->Position() + 1.5708)) + "\n";
        // // R_SHOULDER_ROLL
        // string += std::to_string((float) this->joints[18]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[18]->Position()) + "\n";
        // // L_SHOULDER_ROLL
        // string += std::to_string((float) this->joints[7]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[7]->Position()) + "\n";
        // // R_ELBOW
        // string += std::to_string((float) this->joints[19]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[19]->Position()) + "\n";
        // // L_ELBOW
        // string += std::to_string((float) this->joints[8]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[8]->Position()) + "\n";
        // // R_HIP_YAW
        // string += std::to_string((float) this->joints[11]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[11]->Position()) + "\n";
        // // L_HIP_YAW
        // string += std::to_string((float) this->joints[0]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[0]->Position()) + "\n";
        // // R_HIP_ROLL
        // string += std::to_string((float) this->joints[12]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[12]->Position()) + "\n";
        // // L_HIP_ROLL
        // string += std::to_string((float) this->joints[1]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[1]->Position()) + "\n";
        // // R_HIP_PITCH
        // string += std::to_string((float) this->joints[13]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[13]->Position()) + "\n";
        // // L_HIP_PITCH
        // string += std::to_string((float) this->joints[2]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[2]->Position()) + "\n";
        // // R_KNEE
        // string += std::to_string((float) this->joints[14]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[14]->Position()) + "\n";
        // // L_KNEE
        // string += std::to_string((float) this->joints[3]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[3]->Position()) + "\n";
        // // R_ANKLE_PITCH
        // string += std::to_string((float) this->joints[15]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[15]->Position()) + "\n";
        // // L_ANKLE_PITCH
        // string += std::to_string((float) this->joints[4]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[4]->Position()) + "\n";
        // // R_ANKLE_ROLL
        // string += std::to_string((float) this->joints[16]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[16]->Position()) + "\n";
        // // L_ANKLE_ROLL
        // string += std::to_string((float) this->joints[5]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[5]->Position()) + "\n";
        // // HEAD_YAW
        // string += std::to_string((float) this->joints[9]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[9]->Position()) + "\n";
        // // HEAD_PITCH
        // string += std::to_string((float) this->joints[10]->GetVelocity(0)) + "\n";
        // string += std::to_string((float) this->joints[10]->Position()) + "\n";
        // // std::cerr << "HeadPitch vel: " << (float)this->joints[10]->GetVelocity(0)
        // //	<< ", HeadPitch pos: " << (float)this->joints[10]->Position() << std::endl;

        // ignition::math::Vector3d angularVel = this->imuSensor->AngularVelocity();
        // ignition::math::Vector3d linearAcc  = this->imuSensor->LinearAcceleration();

        // string += std::to_string(angularVel.X()) + "\n" + std::to_string(angularVel.Y()) + "\n"
        //           + std::to_string(angularVel.Z()) + "\n" + std::to_string(linearAcc.X()) + "\n"
        //           + std::to_string(linearAcc.Y()) + "\n" + std::to_string(linearAcc.Z()) + "\n";

        // double x, y, z;
        // ignition::math::Pose3d pose;
        // pose                         = this->model->WorldPose();
        // ignition::math::Vector3d pos = pose.Pos();
        // x                            = pos.X();  // x coordinate
        // y                            = pos.Y();  // y coordinate
        // z                            = pos.Z();  // z coordinate

        // string += std::to_string((float) x) + "\n" + std::to_string((float) y) + "\n" + std::to_string((float) z);

        // jointStatus.set_data(string);
        // return jointStatus;

        // if (!pub.Publish(GetRobotStatus())) std::cerr << "Error publishing to topic [topicStatus]" << std::endl;
    }

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the imu sensor
    sensors::ImuSensorPtr imu_sensor;

    // Pointer to the joints
    std::vector<physics::JointPtr> joints;

    // A command queue to store the incoming servo targets before applying them to the hardware
    std::mutex command_mutex;
    std::vector<message::motion::ServoTarget> command_queue;

    // A PID controller for the joints
    common::PID pid;

    // Holds the callback from gazebo
    event::ConnectionPtr update_connection;

    // The last time we sent a packet so we can rate limit to about 100hz
    std::chrono::steady_clock::time_point last_update;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin
GZ_REGISTER_MODEL_PLUGIN(NUbotsIgusPlugin)

}  // namespace gazebo
