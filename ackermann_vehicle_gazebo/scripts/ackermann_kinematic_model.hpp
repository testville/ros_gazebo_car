#include "movment_model.hpp"
#include "robot_state.hpp"
namespace ackermann_kinematic_model { 

    class AckermannKinematicModel : public movment_model::MovmentModel<robot_state::State, float> {
        public: 
            robot_state::State* solveRK(robot_state::State* pose, float* steeringAngle)
            {
                robot_state::State* k1 = this->GetDerivatives(pose, steeringAngle);
                robot_state::State* k2 = this->GetDerivatives(pose + robot_state::State(tf::Vector3(k1->pose.x() / 2, k1->pose.y() / 2, k1->pose.z()), k1->theta / 2), steeringAngle);
                robot_state::State* k3 = this->GetDerivatives(pose + robot_state::State(tf::Vector3(k2->pose.x() / 2, k2->pose.y() / 2, k2->pose.z()), k2->theta / 2), steeringAngle);
                robot_state::State* k4 = this->GetDerivatives(pose + robot_state::State(tf::Vector3(k3->pose.x(), k3->pose.y(), k3->pose.z()), k3->theta), steeringAngle);
                robot_state::State* accK = &((1.0 / 6.0) * this->_deltaTime * ((*k1) + (*k2) * 2.0 + (*k3) * 2.0 + (*k4)));
                return new robot_state::State(tf::Vector3(pose->pose.x() + accK->pose.x(), pose->pose.y() + accK->pose.y(), pose->pose.z() + accK->pose.z()), pose->theta + accK->theta);
            }
            AckermannKinematicModel(float velocity, float carLength, float deltaTime)
            {
                this->_carLength = carLength;
                this->_velocity = velocity;
                this->_deltaTime = deltaTime;
            }
        private:
            float _velocity, _carLength, _deltaTime;
            robot_state::State* GetDerivatives(robot_state::State* state, float* steeringAngle)
            {
                tf::Vector3 pose;
                pose.setX(this->_velocity * cos(state->theta));
                pose.setY(this->_velocity * sin(state->theta));
                pose.setZ(state->pose.z());
                float newSteerignAngle = (*steeringAngle / this->_carLength) * tan(*steeringAngle);
                return new robot_state::State(pose, newSteerignAngle);
            }
    };
}