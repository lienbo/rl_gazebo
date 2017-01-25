#ifndef ROVERMODEL_HPP_
#define ROVERMODEL_HPP_

// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>


namespace gazebo{
    class RoverModel{
        public:
        RoverModel(physics::ModelPtr model, sdf::ElementPtr sdf);
        ~RoverModel();

        void velocityController() const;
        void steeringWheelController();

        bool checkCollision();
        void resetModel();

        void applyAction(const int &action);
        const float getReward( math::Vector3 setpoint ) const;
        const math::Vector3 getDistanceState( math::Vector3 setpoint ) const;
        const math::Vector3 getPositionState() const;
        const math::Quaternion getOrientationState() const;
        const int getVelocityState() const;
        const int getSteeringState() const;
        void saveImage( const unsigned &state_index ) const;
        const unsigned char* getImage() const;
        const unsigned getImageHeight() const;
        const unsigned getImageWidth() const;

        private:
        void loadParameters();
        void initializeContacts();
        void initializeCamera();
        void checkParameterName( const std::string &parameter_name );

        sdf::ElementPtr sdfFile;
        physics::LinkPtr chassisLink;
        physics::ModelPtr modelPtr;
        physics::JointPtr frontLeftJoint, frontRightJoint;
        physics::JointPtr rearLeftJoint, rearRightJoint;

        sensors::CameraSensorPtr cameraPtr;

        typedef std::vector<sensors::ContactSensorPtr> ContactContainer;
        ContactContainer contactPtrs;

        const std::string outputDir;

        // Angles are in radians. Positive is counterclockwise
        int steeringState;
        int velocityState;
    };
}
#endif
