#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <array>
#include <chrono>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <nuclear>

#include "clock_utils.h"
#include "message/platform/gazebo/RawSensors.pb.h"
#include "message/platform/gazebo/ServoTargets.pb.h"
#include "message/platform/gazebo/Torso.pb.h"
#include "nuclear_network.h"

namespace gazebo {
class NUbotsNUgusPlugin : public ModelPlugin {
private:
    const std::array<double, 20> initial_positions = {0.02924, 0.063,  -0.207, 0.25614, -0.24,    -0.07,  0.4,
                                                      0.123,   -2.443, 0.0,    0.0,     -0.02924, -0.063, -0.207,
                                                      0.25614, -0.24,  0.07,   0.4,     -0.123,   -2.443};
    const double initial_gain                      = 40.0;

    const std::array<double, 20> joint_offsets = {0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  // L_SHOULDER_PITCH 6
                                                  1.5708,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  // R_SHOULDER_PITCH 17
                                                  1.5708,
                                                  0.0,
                                                  0.0};


    struct JointID {
        enum Value : uint32_t {
            R_SHOULDER_PITCH = 17,
            L_SHOULDER_PITCH = 6,
            R_SHOULDER_ROLL  = 18,
            L_SHOULDER_ROLL  = 7,
            R_ELBOW          = 19,
            L_ELBOW          = 8,
            R_HIP_YAW        = 11,
            L_HIP_YAW        = 0,
            R_HIP_ROLL       = 12,
            L_HIP_ROLL       = 1,
            R_HIP_PITCH      = 13,
            L_HIP_PITCH      = 2,
            R_KNEE           = 14,
            L_KNEE           = 3,
            R_ANKLE_PITCH    = 15,
            L_ANKLE_PITCH    = 4,
            R_ANKLE_ROLL     = 16,
            L_ANKLE_ROLL     = 5,
            HEAD_YAW         = 9,
            HEAD_PITCH       = 10,
        };
        Value value;
        JointID(const uint32_t& val) : value(static_cast<Value>(val)) {}
        operator uint32_t() const {
            return value;
        }
    };

    struct ServoID {
        enum Value : uint32_t {
            R_SHOULDER_PITCH = 0,
            L_SHOULDER_PITCH = 1,
            R_SHOULDER_ROLL  = 2,
            L_SHOULDER_ROLL  = 3,
            R_ELBOW          = 4,
            L_ELBOW          = 5,
            R_HIP_YAW        = 6,
            L_HIP_YAW        = 7,
            R_HIP_ROLL       = 8,
            L_HIP_ROLL       = 9,
            R_HIP_PITCH      = 10,
            L_HIP_PITCH      = 11,
            R_KNEE           = 12,
            L_KNEE           = 13,
            R_ANKLE_PITCH    = 14,
            L_ANKLE_PITCH    = 15,
            R_ANKLE_ROLL     = 16,
            L_ANKLE_ROLL     = 17,
            HEAD_YAW         = 18,
            HEAD_PITCH       = 19,
        };
        Value value;
        ServoID(const uint32_t& val) : value(static_cast<Value>(val)) {}
        operator uint32_t() const {
            return value;
        }
    };

    const std::array<ServoID, 20> joint_to_servo_id = {
        ServoID::R_SHOULDER_PITCH, ServoID::L_SHOULDER_PITCH, ServoID::R_SHOULDER_ROLL, ServoID::L_SHOULDER_ROLL,
        ServoID::R_ELBOW,          ServoID::L_ELBOW,          ServoID::R_HIP_YAW,       ServoID::L_HIP_YAW,
        ServoID::R_HIP_ROLL,       ServoID::L_HIP_ROLL,       ServoID::R_HIP_PITCH,     ServoID::L_HIP_PITCH,
        ServoID::R_KNEE,           ServoID::L_KNEE,           ServoID::R_ANKLE_PITCH,   ServoID::L_ANKLE_PITCH,
        ServoID::R_ANKLE_ROLL,     ServoID::L_ANKLE_ROLL,     ServoID::HEAD_YAW,        ServoID::HEAD_PITCH,
    };
    const std::array<JointID, 20> servo_id_to_joint = {
        JointID::R_SHOULDER_PITCH, JointID::L_SHOULDER_PITCH, JointID::R_SHOULDER_ROLL, JointID::L_SHOULDER_ROLL,
        JointID::R_ELBOW,          JointID::L_ELBOW,          JointID::R_HIP_YAW,       JointID::L_HIP_YAW,
        JointID::R_HIP_ROLL,       JointID::L_HIP_ROLL,       JointID::R_HIP_PITCH,     JointID::L_HIP_PITCH,
        JointID::R_KNEE,           JointID::L_KNEE,           JointID::R_ANKLE_PITCH,   JointID::L_ANKLE_PITCH,
        JointID::R_ANKLE_ROLL,     JointID::L_ANKLE_ROLL,     JointID::HEAD_YAW,        JointID::HEAD_PITCH,
    };

public:
    // Inherit constructors
    using ModelPlugin::ModelPlugin;

    virtual ~NUbotsNUgusPlugin() {
        // Unbind handles on destruction
        for (auto& h : handles) {
            h.unbind();
        }
    }

    template <typename T>
    using Network = NUClear::dsl::word::Network<T>;

    /**
     * The load function is called by Gazebo when the plugin is inserted into simulation
     * @param model A pointer to the model that this plugin is attached to
     * @param sdf   A pointer to the plugin's SDF element
     */
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

        // Last update is when we load
        last_update = std::chrono::steady_clock::now();

        // Configure the update rate
        update_rate = Per<std::chrono::seconds>(sdf->Get<uint32_t>("update_rate", 100).first);

        // Safety check to see if the SDF file is attached correctly
        if (model->GetJointCount() == 0) {
            gzerr << "Invalid joint count, NUbots NUgus plugin not loaded" << std::endl;
            return;
        }

        // Just output a message for now
        gzdbg << "Attaching an NUgus plugin to model [" << model->GetName() << "]" << std::endl;

        // Store the model pointer for convenience
        this->model = model;

        // Get the imu for this model
        this->imu_sensor = std::dynamic_pointer_cast<sensors::ImuSensor>(
            sensors::get_sensor(model->SensorScopedName("imu_sensor")[0]));

        // Get the joint pointers
        joints = model->GetJoints();

        // Set up the update event
        update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&NUbotsNUgusPlugin::update_robot, this));

        handles.push_back(get_reactor().on<Network<message::platform::gazebo::ServoTargets>>().then(
            [this](const message::platform::gazebo::ServoTargets& msg) {
                // Check that the commands we are getting are for this model instance
                if (msg.model() == this->model->GetName()) {
                    // Store the incoming commands in the queue for the next simulation frame
                    std::lock_guard<std::mutex> lock(command_mutex);
                    for (const auto& c : msg.targets().targets()) {
                        // Convert ServoID to JointID
                        message::motion::ServoTarget joint_target(c);
                        joint_target.set_id(servo_id_to_joint[c.id()]);

                        // Apply offset to joint position
                        joint_target.set_position(c.position() - joint_offsets[c.id()]);

                        // Convert command time to a duration from msg.time()
                        ::google::protobuf::Duration offset;
                        offset.set_seconds(msg.time().seconds() - c.time().seconds());
                        offset.set_nanos(msg.time().nanos() - c.time().nanos());

                        // Set joint command time to be based on the local time plus the duration we just calculated
                        ::google::protobuf::Timestamp local_time(time_point_cast(std::chrono::steady_clock::now()));
                        joint_target.mutable_time()->set_seconds(local_time.seconds() + offset.seconds());
                        joint_target.mutable_time()->set_nanos(local_time.nanos() + offset.nanos());

                        command_queue.push_back(joint_target);
                    }
                }
            }));

        // Apply the P-controller to the joint.
        for (int i = 0; i < joints.size(); i++) {
            // Set an initial p gain
            model->GetJointController()->SetPositionPID(joints[i]->GetScopedName(),
                                                        common::PID(initial_gain, 0.0, 0.0));

            // This will set the initial positions of the robot
            model->GetJointController()->SetPositionTarget(joints[i]->GetScopedName(), initial_positions[i]);
            joints[i]->SetPosition(0, initial_positions[i]);

            joints[i]->SetStopDissipation(0, 0.0);
            joints[i]->SetStiffness(0, 3.25);
            // joints[i]->SetStopStiffness (0, 0.125);
            // joints[i]->Set
            // joints[i]->SetStiffnessDamping(0, 4.0, 0.125, 0.0);
            // joints[i]->Set
            joints[i]->SetEffortLimit(0, 20.0);

            joints[i]->SetStiffnessDamping(0, 3.25, 0.25, joints[i]->Position());
        }
    }

private:
    void apply_joint_command(const message::motion::ServoTarget& target) {
        // Set the joint's gain by applying the
        // P-controller to the joints for positions.
        double gain =
            target.gain() * (target.id() == JointID::R_ANKLE_ROLL || target.id() == JointID::L_ANKLE_ROLL ? 0.7 : 4.0);

        // Only update the controller if the gain changed
        model->GetJointController()->SetPositionPID(joints[target.id()]->GetScopedName(), common::PID(gain));

        auto duration = time_point_cast(target.time()) - std::chrono::steady_clock::now();

        // Calculate the joint's velocity
        double current_position = model->GetJointController()->GetPositions()[joints[target.id()]->GetScopedName()];
        double position_difference =
            M_PI - std::fabs(std::fmod(std::fabs(current_position - target.position()), 2 * M_PI) - M_PI);
        double velocity =
            position_difference
            / (static_cast<double>(duration.count()) / static_cast<double>(std::chrono::steady_clock::period::den));

        // Set the joint's target position.
        model->GetJointController()->SetPositionTarget(joints[target.id()]->GetScopedName(), target.position());

        if (velocity > 0.0) {
            joints[target.id()]->SetVelocityLimit(0, velocity);
        }
        else {
            joints[target.id()]->SetVelocityLimit(0, 3.75);
        }

        joints[target.id()]->SetStiffnessDamping(0, 3.25, 0.25, joints[target.id()]->Position());
        // model->GetJointController()->Update();
    }

    void update_robot() {

        // Run all our commands in the command queue and then clear it
        {
            std::lock_guard<std::mutex> lock(command_mutex);
            for (const auto& c : command_queue) {
                apply_joint_command(c);
            }
            command_queue.resize(0);
        }

        auto now = std::chrono::steady_clock::now();
        if (now - last_update > update_rate) {
            last_update += update_rate;

            auto msg = std::make_unique<message::platform::gazebo::RawSensors>();
            msg->set_model(model->GetName());
            auto sensors = msg->mutable_sensors();
            for (uint32_t i = 0; i < 20; ++i) {
                auto s = sensors->add_servos();
                s->set_presentposition(joints[servo_id_to_joint[i]]->Position() + joint_offsets[servo_id_to_joint[i]]);
                s->set_presentspeed(joints[servo_id_to_joint[i]]->GetVelocity(0));
            }

            ignition::math::Vector3d gyroscope     = imu_sensor->AngularVelocity();
            ignition::math::Vector3d accelerometer = imu_sensor->LinearAcceleration();
            auto gyro                              = sensors->mutable_gyroscope();
            auto acc                               = sensors->mutable_accelerometer();
            gyro->set_x(gyroscope.X());
            gyro->set_y(gyroscope.Y());
            gyro->set_z(gyroscope.Z());
            acc->set_x(accelerometer.X());
            acc->set_y(accelerometer.Y());
            acc->set_z(accelerometer.Z());

            get_reactor().emit<NUClear::dsl::word::emit::Network>(msg);

            auto torso_msg = std::make_unique<message::platform::gazebo::Torso>();
            torso_msg->set_model(model->GetName());

            // Get robot pose and convert to protobuf message
            ignition::math::Pose3d pose(model->WorldPose());
            ignition::math::Matrix3d rotation(pose.Rot());
            ignition::math::Vector3d translation(pose.Pos());
            torso_msg->mutable_htw()->mutable_x()->set_x(rotation(0, 0));
            torso_msg->mutable_htw()->mutable_x()->set_y(rotation(1, 0));
            torso_msg->mutable_htw()->mutable_x()->set_z(rotation(2, 0));
            torso_msg->mutable_htw()->mutable_x()->set_t(0.0);
            torso_msg->mutable_htw()->mutable_y()->set_x(rotation(0, 1));
            torso_msg->mutable_htw()->mutable_y()->set_y(rotation(1, 1));
            torso_msg->mutable_htw()->mutable_y()->set_z(rotation(2, 1));
            torso_msg->mutable_htw()->mutable_y()->set_t(0.0);
            torso_msg->mutable_htw()->mutable_z()->set_x(rotation(0, 2));
            torso_msg->mutable_htw()->mutable_z()->set_y(rotation(1, 2));
            torso_msg->mutable_htw()->mutable_z()->set_z(rotation(2, 2));
            torso_msg->mutable_htw()->mutable_z()->set_t(0.0);
            torso_msg->mutable_htw()->mutable_t()->set_x(translation.X());
            torso_msg->mutable_htw()->mutable_t()->set_y(translation.Y());
            torso_msg->mutable_htw()->mutable_t()->set_z(translation.Z());
            torso_msg->mutable_htw()->mutable_t()->set_t(1.0);

            // Get linear and angular velocties and accelerations and convert to protobuf message
            ignition::math::Vector3d omegaTw(model->WorldAngularVel());
            ignition::math::Vector3d alphaTw(model->WorldAngularAccel());
            ignition::math::Vector3d vTw(model->WorldLinearVel());
            ignition::math::Vector3d aTw(model->WorldLinearAccel());
            torso_msg->mutable_omegatw()->set_x(omegaTw.X());
            torso_msg->mutable_omegatw()->set_y(omegaTw.Y());
            torso_msg->mutable_omegatw()->set_z(omegaTw.Z());
            torso_msg->mutable_alphatw()->set_x(alphaTw.X());
            torso_msg->mutable_alphatw()->set_y(alphaTw.Y());
            torso_msg->mutable_alphatw()->set_z(alphaTw.Z());
            torso_msg->mutable_vtw()->set_x(vTw.X());
            torso_msg->mutable_vtw()->set_y(vTw.Y());
            torso_msg->mutable_vtw()->set_z(vTw.Z());
            torso_msg->mutable_atw()->set_x(aTw.X());
            torso_msg->mutable_atw()->set_y(aTw.Y());
            torso_msg->mutable_atw()->set_z(aTw.Z());

            get_reactor().emit<NUClear::dsl::word::emit::Network>(torso_msg);
        }
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

    // The last time we sent a packet so we can rate limit
    std::chrono::steady_clock::time_point last_update;

    // Rate at which to send update messages over the network
    std::chrono::steady_clock::duration update_rate;

    // Reaction handles we have made so we can unbind when we are destructed
    std::vector<NUClear::threading::ReactionHandle> handles;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin
GZ_REGISTER_MODEL_PLUGIN(NUbotsNUgusPlugin)

}  // namespace gazebo
