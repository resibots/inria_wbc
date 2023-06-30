#ifndef INRIA_UTILS_VIVETRACKING_HPP
#define INRIA_UTILS_VIVETRACKING_HPP

#include <map>
#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace inria {

/**
 * ViveTracking
 *
 * ROS subscriber and frame calibration and interface 
 * for the Vive Tracking system.
 * Uses ROS topic published by inria_docker_vive_tracker.
 */
class ViveTracking
{
    public:

        /**
         * Structure for received and processed 
         * data of Controller devices
         */
        struct Device_t {
            //Vive last sequence number
            uint32_t seq;
            //Vive and local timestamp of last received pose
            ros::Time timeVive;
            ros::Time timeLocal;
            //Measured raw pose in world
            Eigen::Vector3d posRaw;
            Eigen::Matrix3d matRaw;
            //Is the last received pose valid
            bool isLastValid;
            //Is the pose valid, fresh and updated
            bool isValid;
            //Is the buttons pushed
            bool isButtonTrigger;
            bool isButtonPad;
            bool isButtonGrip;
            //Pose of hand on the controller handle in world
            Eigen::Vector3d posHand;
            Eigen::Matrix3d matHand;
            //Vive time at last transition 
            //from not valid to valid
            ros::Time timeViveAtValid;
        };

        /**
         * Empty initialization
         */
        ViveTracking();

        /**
         * Deallocation
         */
        ~ViveTracking();

        /**
         * Initialize ROS node and subscriber.
         *
         * @param ipMaster Textual IP address to ROS master.
         * @param ipNode Textual IP address for the node being created.
         * @param isOptional If true, init() won't fail if ROS master
         * is not available and false is returned.
         * @return true if initialization is successful.
         */
        bool init(
            const std::string& ipMaster = "127.0.0.1",
            const std::string& ipNode = "127.0.0.1",
            bool isOptional = false);

        /**
         * @return true if the subscriber is initialized
         */
        bool isInit() const;

        /**
         * Receive data and update processing.
         *
         * @return true if pose data has been updated.
         */
        bool update();

        /**
         * Access to raw and processed data structure
         */
        const std::map<std::string, Device_t>& get() const;

    private:

        /**
         * ROS instances
         */
        ros::NodeHandle* _handle;
        ros::Subscriber _subPose;
        ros::Subscriber _subJoy;

        /**
         * Data container
         */
        std::map<std::string, Device_t> _container;

        /**
         * True if device data has been updated during update()
         */
        bool _isUpdated;
};

}

#endif


