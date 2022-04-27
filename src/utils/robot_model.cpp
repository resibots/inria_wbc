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

} // namespace utils
} // namespace inria_wbc
