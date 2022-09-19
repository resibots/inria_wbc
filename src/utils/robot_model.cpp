#include <inria_wbc/utils/robot_model.hpp>
#include <inria_wbc/exceptions.hpp>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Eigenvalues>

namespace inria_wbc {
namespace utils {

    RobotModel::RobotModel(const std::string& urdf_path, const RobotModel::Configuration& config)
        :  _config(config)
    {
        if (_config.is_floating_base) {
            if (!_config.fb_joint_name.empty()) {
                _fb_joint_name = config.fb_joint_name; // floating base joint already in urdf
                pinocchio::urdf::buildModel(urdf_path, _model, _config.verbose);
            } else {
                pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), _model, _config.verbose);
                _fb_joint_name = "root_joint";
            }
        } else {
            _fb_joint_name = "";
            pinocchio::urdf::buildModel(urdf_path, _model, _config.verbose);
        }

        _data = pinocchio::Data(_model);
    }

    RobotModel::RobotModel(const pinocchio::Model& model, const RobotModel::Configuration& config)
     :  _config(config),
        _model(model),
        _data(model)
    {
        _fb_joint_name = "";
        if(_config.is_floating_base)
            _fb_joint_name = (_config.fb_joint_name.empty() ? std::string("root_joint") : _config.fb_joint_name);
    }

    pinocchio::SE3 RobotModel::position(std::string joint_name)
    {
        IWBC_ASSERT(_model.existJointName(joint_name), "Joint " + joint_name + " does not exist in model.");
        return _data.oMi[_model.getJointId(joint_name)];
    }

    pinocchio::SE3 RobotModel::frame_position(std::string frame_name)
    {
        IWBC_ASSERT(_model.existFrame(frame_name), "Frame " + frame_name + " does not exist in model.");
        auto index = _model.getFrameId(frame_name);
        return _data.oMi[_model.frames[index].parent].act(_model.frames[index].placement);
    }

    std::vector<std::string> RobotModel::frame_names()
    {
        std::vector<std::string> frames;
        std::transform(_model.frames.begin(), _model.frames.end(), std::back_inserter(frames),
            [](const pinocchio::Frame& f) { return f.name; }
        );
        return frames;
    }

    void RobotModel::update(const Eigen::VectorXd& q, bool update_dynamics, bool update_jacobians)
    {
        update(q, Eigen::VectorXd::Zero(_model.nv), Eigen::VectorXd::Zero(_model.nv), update_dynamics, update_jacobians); 
    }

    void RobotModel::update(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, bool update_dynamics, bool update_jacobians)
    {
        update(q, dq, Eigen::VectorXd::Zero(_model.nv), update_dynamics, update_jacobians);
    }

    void RobotModel::update(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq, bool update_dynamics, bool update_jacobians)
    {
        _dyn_updated = false;
        _jac_updated = false;

        // update kynematics
        pinocchio::forwardKinematics(_model, _data, q, dq);
        pinocchio::centerOfMass(_model, _data, q, dq, ddq, false); // false -> not computed for subtrees

        if(update_jacobians)
        {
            pinocchio::computeJointJacobians(_model, _data);
            pinocchio::jacobianCenterOfMass(_model, _data, false); // compute also joint jacobians ?
            pinocchio::computeJointKinematicHessians(_model, _data);
            _jac_updated = true;
        }
        
        if(update_dynamics)
        {
            pinocchio::crba(_model, _data, q);
            _data.M.triangularView<Eigen::StrictlyLower>() = _data.M.transpose().triangularView<Eigen::StrictlyLower>();

            pinocchio::nonLinearEffects(_model, _data, q, dq);
            pinocchio::computeGeneralizedGravity(_model, _data, q);

            //pinocchio::ccrba(_model, _data, q, dq);  // centroidal momentum mtx, rigid body inertia, centroidal momenta
            _dyn_updated = true;
        }

        pinocchio::updateFramePlacements(_model, _data);
    }

    pinocchio::Data::Matrix6x RobotModel::jacobian(const std::string& joint_name, const pinocchio::ReferenceFrame& reference_frame)
    {
        if (!_model.existJointName(joint_name))
            throw IWBC_EXCEPTION("Joint name ", joint_name, "is not in model");
        IWBC_ASSERT(_jac_updated, "Jacobians have not been updated in RobotModel::update_model.");

        pinocchio::Model::Index id = _model.getJointId(joint_name);
        pinocchio::Data::Matrix6x J(6, _model.nv);
        J.fill(0);
        pinocchio::getJointJacobian(_model, _data, id, reference_frame, J);
        return J;
    }

    pinocchio::Data::Tensor3x RobotModel::hessian(const std::string& joint_name, const pinocchio::ReferenceFrame& reference_frame)
    {
        if (!_model.existJointName(joint_name))
            throw IWBC_EXCEPTION("Joint name ", joint_name, "is not in model");
        IWBC_ASSERT(_jac_updated, "Jacobians have not been updated in RobotModel::update_model.");

        pinocchio::Model::Index id = _model.getJointId(joint_name);
        return pinocchio::getJointKinematicHessian(_model, _data, id, reference_frame);
    }

    Eigen::VectorXd RobotModel::compute_rnea_double_support(const std::map<std::string, Eigen::MatrixXd>& sensor_data,
        const Eigen::VectorXd& q, 
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& a, 
        bool add_foot_mass,
        const std::string& left_ft_frame,
        const std::string& right_ft_frame,
        const std::string& left_sole_frame,
        const std::string& right_sole_frame)
    {
        pinocchio::computeJointJacobians(_model, _data, q);
        pinocchio::updateFramePlacements(_model, _data);

        //compute torques with no external forces
        pinocchio::rnea(_model, _data, q, v, a);
        Eigen::VectorXd tau(_data.tau);

        //get F/T sensor data from both feet
        Eigen::Vector3d right_force = Eigen::Vector3d::Zero();
        Eigen::Vector3d right_torque = Eigen::Vector3d::Zero();
        Eigen::Vector3d left_force = Eigen::Vector3d::Zero();
        Eigen::Vector3d left_torque = Eigen::Vector3d::Zero();

       if((sensor_data.find("lf_force") == sensor_data.end()) || (sensor_data.find("rf_force") == sensor_data.end()) ||
          (sensor_data.find("lf_torque") == sensor_data.end()) || (sensor_data.find("rf_torque") == sensor_data.end()))
            throw IWBC_EXCEPTION("when FT is missing in fext_map");

        //Get data for the right foot
        right_force = sensor_data.at("rf_force").col(0).head(3);
        right_torque = sensor_data.at("rf_torque").col(0).head(3);
        std::string right_frame = right_ft_frame;

        //Add the mass of the foot to sensor_data
        if(add_foot_mass)
        {
            if (!_model.existFrame(right_ft_frame))
                throw IWBC_EXCEPTION("Frame name ", right_ft_frame, "is not in model");
            if (!_model.existFrame(right_sole_frame))
                throw IWBC_EXCEPTION("Frame name ", right_sole_frame, "is not in model");

            auto ankle_world = _data.oMi[_model.getJointId(right_ft_frame)];
            auto sole_world = _data.oMf[_model.getFrameId(right_sole_frame)];

            float mass_to_add = _model.inertias[_model.getJointId(right_ft_frame)].mass();
            //TODO hypothesis foot is on planar surface here ?
            right_force -= mass_to_add * _model.gravity.linear();
            right_torque += (ankle_world.translation() - sole_world.translation()).cross(right_force);
            right_frame = right_sole_frame;
        }

        //Get data for the left foot
        left_force = sensor_data.at("lf_force").col(0).head(3);
        left_torque = sensor_data.at("lf_torque").col(0).head(3);
        std::string left_frame = left_ft_frame;
        
        //Add the mass of the foot to sensor_data
        if(add_foot_mass)
        {
            if (!_model.existFrame(left_ft_frame))
                throw IWBC_EXCEPTION("Frame name ", left_ft_frame, "is not in model");
            if (!_model.existFrame(left_sole_frame))
                throw IWBC_EXCEPTION("Frame name ", left_sole_frame, "is not in model");

            auto ankle_world = _data.oMi[_model.getJointId(left_ft_frame)];
            auto sole_world = _data.oMf[_model.getFrameId(left_sole_frame)];

            float mass_to_add = _model.inertias[_model.getJointId(left_ft_frame)].mass();
            //TODO hypothesis foot is on planar surface here ?
            left_force -= mass_to_add * _model.gravity.linear();
            left_torque += (ankle_world.translation() - sole_world.translation()).cross(left_force);
            left_frame = left_sole_frame;
        }

        //Compute the torque from external forces J^transpose*force
        pinocchio::Data::Matrix6x J_right(pinocchio::Data::Matrix6x(6, _model.nv));
        J_right.fill(0.);
        if (_model.existFrame(right_frame))
            pinocchio::getFrameJacobian(_model, _data, _model.getFrameId(right_frame), pinocchio::LOCAL, J_right);

        pinocchio::Data::Matrix6x J_left(pinocchio::Data::Matrix6x(6, _model.nv));
        J_left.fill(0.);
        if (_model.existFrame(left_frame))
            pinocchio::getFrameJacobian(_model, _data, _model.getFrameId(left_frame), pinocchio::LOCAL, J_left);

        pinocchio::Force f_right(right_force, right_torque);
        pinocchio::Force f_left(left_force, left_torque);
        tau -= J_right.transpose() * f_right.toVector();
        tau -= J_left.transpose() * f_left.toVector();

        return tau;

    };


} // namespace utils
} // namespace inria_wbc
