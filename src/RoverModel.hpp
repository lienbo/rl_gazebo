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
        RoverModel( physics::ModelPtr model, sdf::ElementPtr sdf );
        ~RoverModel();

        void velocityController() const;
        void steeringWheelController() const;

        bool checkCollision() const;
        void setOriginAndDestination( const std::vector<math::Pose> &initial_pos,
                                      const std::vector<math::Vector3> &destination_pos );

        const unsigned bestAction() const;
        void applyAction(const unsigned &action);
        const bool isTerminalState();
        const bool isDistancing();

        void endStep();
        void resetModel();

        const unsigned getNumActions() const;
        const float getAngletoDestination() const;
        const float getDestinationDistance() const;
        const float getReward() const;

        const math::Vector3 getDistanceState() const;
        const math::Vector3 getPositionState() const;
        const math::Vector3 getEulerAnglesState() const;
        const math::Quaternion getOrientationState() const;
        const int getVelocityState() const;
        const int getSteeringState() const;
        std::vector<float> getState() const;
        void printState( const std::vector<float> &observed_state ) const;

        void saveImage( const unsigned &state_index ) const;
        const unsigned char* getImage() const;
        const unsigned getImageHeight() const;
        const unsigned getImageWidth() const;

        common::Time getActionInterval() const;

        private:
        void loadParameters();
        void checkParameterName( const std::string &parameter_name );
        void initializeContacts();
        void initializeCamera();
        void selectSimulationSpeed( const std::string speed = "normal");

        enum Action{ DO_NOTHING, INCREASE_SPEED, REDUCE_SPEED, TURN_LEFT, TURN_RIGHT, NUM_ACTIONS = 5 };

        typedef std::vector<sensors::ContactSensorPtr> ContactContainer;
        ContactContainer contactPtrs;

        sdf::ElementPtr sdfFile;
        physics::ModelPtr modelPtr;
        common::Time actionInterval;

        math::Vector3 setPoint;
        std::vector<math::Pose> initialPos;
        std::vector<math::Vector3> destinationPos;

        physics::LinkPtr chassisLink;
        physics::JointPtr frontLeftJoint, frontRightJoint;
        physics::JointPtr rearLeftJoint, rearRightJoint;
        sensors::CameraSensorPtr cameraPtr;

        std::default_random_engine generator;

        const std::string outputDir;

        // Angles are in radians. Positive is counterclockwise
        int steeringState;
        int velocityState;

        unsigned terminalStateCounter, distanceCounter;
        const float terminalDistance;
        float lastDistance;
        float simulationFactor;
    };
}
#endif
