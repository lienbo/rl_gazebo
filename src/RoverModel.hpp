#ifndef ROVERMODEL_HPP_
#define ROVERMODEL_HPP_

// Copyright © 2017 Thomio Watanabe
// Universidade de Sao Paulo
// Laboratorio de robotica movel
// January 2017

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <random>


namespace gazebo{
    class RoverModel{
        public:
        RoverModel(physics::ModelPtr model, sdf::ElementPtr sdf, math::Vector3 );
        ~RoverModel();

        void velocityController() const;
        void steeringWheelController();

        bool checkCollision();
        void resetModel( bool change_pose = false );

        void applyAction(const int &action);
        const float getReward() const;
        const bool isTerminalState() const;

        const math::Vector3 getDistanceState() const;
        const math::Vector3 getPositionState() const;
        const math::Quaternion getOrientationState() const;
        const int getVelocityState() const;
        const int getSteeringState() const;
        void saveImage( const unsigned &state_index ) const;
        const unsigned char* getImage() const;
        const unsigned getImageHeight() const;
        const unsigned getImageWidth() const;
        const unsigned getNumActions() const;

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

        math::Vector3 setPoint;
        std::vector<math::Pose> initialPos;
        std::vector<math::Vector3> destinationPos;
        std::default_random_engine generator;
        std::uniform_int_distribution<int> uniformDist;

        // Angles are in radians. Positive is counterclockwise
        int steeringState;
        int velocityState;
    };
}
#endif
