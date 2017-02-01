#ifndef ROVER_MODEL_HPP_
#define ROVER_MODEL_HPP_

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
        void resetModel();
        void resetModel( std::vector<math::Pose> initial_pos, std::vector<math::Vector3> destination_pos );

        void applyAction(const unsigned &action);
        const float getDestinationDistance() const;
        const float getReward() const;
        const bool isTerminalState();
        const bool isDistancing();

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
        enum Action{ DO_NOTHING, FORWARD, BACKWARD, TURN_RIGHT, TURN_LEFT, NUM_ACTIONS = 5 };

        void loadParameters();
        void initializeContacts();
        void initializeCamera();
        void checkParameterName( const std::string &parameter_name );

        const std::string outputDir;
        math::Vector3 setPoint;

        sdf::ElementPtr sdfFile;
        physics::ModelPtr modelPtr;

        physics::LinkPtr chassisLink;
        physics::JointPtr frontLeftJoint, frontRightJoint;
        physics::JointPtr rearLeftJoint, rearRightJoint;
        sensors::CameraSensorPtr cameraPtr;

        std::default_random_engine generator;

        typedef std::vector<sensors::ContactSensorPtr> ContactContainer;
        ContactContainer contactPtrs;

        // Angles are in radians. Positive is counterclockwise
        int steeringState;
        int velocityState;

        unsigned terminalStateCounter, distanceCounter;
        float lastDistance;
    };
}
#endif
