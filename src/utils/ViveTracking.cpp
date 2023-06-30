#include <stdexcept>
#include <sensor_msgs/MultiDOFJointState.h>
#include <sensor_msgs/Joy.h>
#include <inria_wbc/utils/ViveTracking.hpp>

namespace inria {

ViveTracking::ViveTracking() :
    _handle(nullptr),
    _subPose(),
    _subJoy(),
    _container(),
    _isUpdated(false)
{
}
        
ViveTracking::~ViveTracking()
{
    if (_handle != nullptr) {
        ros::shutdown();
        delete _handle;
        _handle = nullptr;
    }
}

bool ViveTracking::init(
    const std::string& ipMaster,
    const std::string& ipNode,
    bool isOptional)
{
    //Check if already initialized
    if (_handle != nullptr) {
        throw std::logic_error(
            "inria::ViveTracking::init: Already initialized");
    }

    //ROS initialization
    std::map<std::string, std::string> remappingROSParams;
    remappingROSParams["__master"] = "http://" + ipMaster + ":11311/";
    remappingROSParams["__ip"] = ipNode;
    ros::init(
        remappingROSParams, "inria_ViveTracking", 
        ros::init_options::NoSigintHandler);
    //Check if ROS is running
    if (!ros::master::check()) {
        if (isOptional) {
            return false;
        } else {
            throw std::logic_error(
                "inria::ViveTracking::init: ROS master is not running");
        }
    }
    _handle = new ros::NodeHandle();
    
    //Transform from Vive frame convention up is Y to up is Z
    const Eigen::Matrix3d rotationViveToWorld = 
        Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX())
        .toRotationMatrix();

    //Define receiving callback for pose
    auto callback_pose = [this,rotationViveToWorld]
        (const sensor_msgs::MultiDOFJointState::ConstPtr& msg) {
        for (size_t i=0;i<msg->joint_names.size();i++) {
            //Extract data
            const std::string& label = msg->joint_names[i];
            Eigen::Vector3d pos;
            Eigen::Quaterniond quat;
            pos.x() = msg->transforms[i].translation.x;
            pos.y() = msg->transforms[i].translation.y;
            pos.z() = msg->transforms[i].translation.z;
            quat.x() = msg->transforms[i].rotation.x;
            quat.y() = msg->transforms[i].rotation.y;
            quat.z() = msg->transforms[i].rotation.z;
            quat.w() = msg->transforms[i].rotation.w;
            quat.normalize();
            Eigen::Matrix3d mat = quat.toRotationMatrix();
            bool isConnected = (msg->wrench[i].force.x > 0.0);
            bool isPoseValid = (msg->wrench[i].force.y > 0.0);
            int resultTracking = (int)(msg->wrench[i].force.z);
            int32_t type = (int32_t)(msg->wrench[i].torque.x);
            uint32_t timeSec = (uint32_t)(msg->wrench[i].torque.y);
            uint32_t timeNSec = (uint32_t)(msg->wrench[i].torque.z);
            //Check inputs for NaN
            for (size_t j=0;j<3;j++) {
                if (std::isnan(pos(j))) {
                    throw std::logic_error(
                        "inria::ViveTracking::update(); NaN received.");
                }
            }
            for (size_t j=0;j<3;j++) {
                for (size_t l=0;l<3;l++) {
                    if (std::isnan(mat(j,l))) {
                        throw std::logic_error(
                            "inria::ViveTracking::update(); NaN received.");
                    }
                }
            }
            //Change from Vive frame convention up is Y to up is Z
            pos = rotationViveToWorld*pos;
            mat = rotationViveToWorld*mat;
            //Retrieve only Controller device type
            if (type == 2) {
                //Container initialization
                if (_container.count(label) == 0) {
                    _container.insert(std::make_pair(label, Device_t()));
                    Device_t& devInit = _container.at(label);
                    devInit.isLastValid = false;
                    devInit.isValid = false;
                    devInit.isButtonTrigger = false;
                    devInit.isButtonPad = false;
                    devInit.isButtonGrip = false;
                    devInit.posHand = Eigen::Vector3d::Zero();
                    devInit.matHand = Eigen::Matrix3d::Identity();
                }
                //Assign data
                Device_t& dev = _container.at(label);
                dev.seq = msg->header.seq;
                dev.timeVive.sec = timeSec;
                dev.timeVive.nsec = timeNSec;
                dev.timeLocal = ros::Time::now();
                dev.posRaw = pos;
                dev.matRaw = mat;
                bool isLastValid = 
                    isConnected && isPoseValid && (resultTracking == 200);
                if (!dev.isLastValid && isLastValid) {
                    dev.timeViveAtValid = dev.timeVive;
                }
                dev.isLastValid = isLastValid;
                _isUpdated = true;
            }
        }
    };
    _subPose = _handle->subscribe<sensor_msgs::MultiDOFJointState>(
        "vive_pose", 100, callback_pose, nullptr, ros::TransportHints().tcpNoDelay());

    //Define receiving callback for buttons
    auto callback_joy = [this](const sensor_msgs::Joy::ConstPtr& msg) {
        const std::string& label = msg->header.frame_id;
        //Check buttons status
        if (_container.count(label) > 0) {
            _container.at(label).isButtonTrigger = (msg->buttons[33] > 0);
            _container.at(label).isButtonPad = (msg->buttons[32] > 0);
            _container.at(label).isButtonGrip = (msg->buttons[2] > 0);
        }
    };
    _subJoy = _handle->subscribe<sensor_msgs::Joy>(
        "vive_joy", 100, callback_joy, nullptr, ros::TransportHints().tcpNoDelay());

    return true;
}
        
bool ViveTracking::isInit() const
{
    return (_handle != nullptr);
}

bool ViveTracking::update()
{
    //Check if initialized
    if (_handle == nullptr) {
        throw std::logic_error(
            "inria::ViveTracking::init: Not initialized");
    }

    //Receive data from ROS and call callbacks
    _isUpdated = false;
    ros::spinOnce();
    
    //Transforms configuration
    const Eigen::Vector3d translationControllerToHand =
        Eigen::Vector3d(0.0, 0.0, 0.11);
    const Eigen::Matrix3d rotationControllerToHand = 
        Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX())
        .toRotationMatrix();

    //Data processing
    for (auto& it : _container) {
        //Set as invalid if last received pose is too old
        //or if it is not valid since enough time
        if (
            it.second.isLastValid &&
            (ros::Time::now() - it.second.timeLocal).toSec() < 0.5 /*&&
            (it.second.timeVive - it.second.timeViveAtValid).toSec() > 1.0*/
        ) {
            it.second.isValid = true;
        } else {
            it.second.isValid = false;
            it.second.isButtonTrigger = false;
            it.second.isButtonPad = false;
            it.second.isButtonGrip = false;
        }
        //Compute hand pose on the controller's 
        //handle transformed from raw pose
        it.second.posHand = 
            it.second.posRaw + it.second.matRaw*translationControllerToHand;
        it.second.matHand = 
            it.second.matRaw*rotationControllerToHand;
    }

    return _isUpdated;
}
        
const std::map<std::string, ViveTracking::Device_t>& ViveTracking::get() const
{
    return _container;
}

}
