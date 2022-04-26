#ifndef IWBC_UTILS_ROBOT_MODEL_HPP
#define IWBC_UTILS_ROBOT_MODEL_HPP

/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <inria_wbc/exceptions.hpp>

#include <string>


namespace inria_wbc {
namespace utils {

    class RobotModel {

    public:
        struct Configuration 
        {
            bool is_floating_base;
            std::string fb_joint_name;

            bool verbose;
        };

        RobotModel() = default;
        RobotModel(const std::string& urdf_path, const Configuration& config);
        RobotModel(const pinocchio::Model& model, const Configuration& config);

        RobotModel& operator=(const RobotModel& o) = delete;
        virtual ~RobotModel() = default;

        const Configuration& configuration() const { return _config; }
        inline bool is_floating_base() { return _config.is_floating_base; }
        inline void set_world_gravity(const Eigen::Vector3d& g) { _model.gravity = pinocchio::Motion(g, Eigen::Vector3d::Zero()); };

        size_t nq() const { return _model.nq; }
        size_t nv() const { return _model.nv; }

        inline const Eigen::MatrixXd& inertia_matrix() { IWBC_ASSERT(_dyn_updated, "Dynamic has not been updated."); return _data.M; }
        inline const Eigen::VectorXd& bias_vector() { IWBC_ASSERT(_dyn_updated, "Dynamic has not been updated."); return _data.nle; }
        inline const Eigen::VectorXd& gravity_vector() {  IWBC_ASSERT(_dyn_updated, "Dynamic has not been updated."); return _data.g; }
        inline const Eigen::Vector3d& com() { return _data.com[0]; }
        inline const Eigen::Vector3d& com_velocity() { return _data.vcom[0]; }

        pinocchio::SE3 position(std::string joint_name);
        pinocchio::SE3 frame_position(std::string frame_name);

        const pinocchio::Model& model() const { return _model; }
        pinocchio::Model& model() { return _model; } // a bit dangerous, maybe better to create a copy and recreate a RobotModel

        const std::vector<std::string>& joint_names() const { return _model.names; }
        std::vector<std::string> frame_names();

        void update(const Eigen::VectorXd& q, bool update_dynamics = false, bool update_jacobians = false);
        void update(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, bool update_dynamics = false, bool update_jacobians = false);
        void update(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq, bool update_dynamics = false, bool update_jacobians = false);

        pinocchio::Data::Matrix6x jacobian(const std::string& joint_name, const pinocchio::ReferenceFrame& reference_frame = pinocchio::WORLD);
        pinocchio::Data::Tensor3x hessian(const std::string& joint_name, const pinocchio::ReferenceFrame& reference_frame = pinocchio::WORLD);

    protected:

        Configuration _config;
        
        pinocchio::Model _model;
        pinocchio::Data _data;
        
        std::string _fb_joint_name; // floating base joint name

        bool _dyn_updated = false;
        bool _jac_updated = false;
    };

} //namespace utils
} // namespace inria_wbc

#endif // IWBC_UTILS_ROBOT_MODEL_HPP
