#ifndef ROVERMODEL_HPP
#define ROVERMODEL_HPP

// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


namespace gazebo{
    class RoverModel{
        public:
        RoverModel(physics::ModelPtr model, sdf::ElementPtr sdf);
        ~RoverModel();

        void velocityController() const;
        void steeringWheelController();

        void applyAction(const int &action);
        const float getReward( math::Vector3 set_point ) const;

        const math::Vector3 getPositionState() const;
        const math::Quaternion getOrientationState() const;
        const int getVelocityState() const;
        const int getSteeringState() const;

        private:
        void loadParameters();
        void checkParameterName( const std::string &parameter_name );

        sdf::ElementPtr sdfFile;
        physics::LinkPtr chassisLink;
        physics::ModelPtr carinaModel;
        physics::JointPtr frontLeftJoint, frontRightJoint;
        physics::JointPtr rearLeftJoint, rearRightJoint;

        // Angles are in radians. Positive is counterclockwise
        int steeringState;
        int velocityState;
    };
}
#endif
