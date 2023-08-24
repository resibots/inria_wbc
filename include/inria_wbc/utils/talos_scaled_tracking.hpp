#ifndef TALOS_SCALED_TRACKING_HPP
#define TALOS_SCALED_TRACKING_HPP

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <fstream>

#include <dart/collision/CollisionObject.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/BodyNode.hpp>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/imu.hpp>
#include <robot_dart/sensor/torque.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/robot_dart/damages.hpp"
#include "inria_wbc/robot_dart/external_collision_detector.hpp"
#include "inria_wbc/robot_dart/self_collision_detector.hpp"
#include "inria_wbc/robot_dart/utils.hpp"
#include "inria_wbc/trajs/loader.hpp"
#include "inria_wbc/trajs/saver.hpp"
#include "inria_wbc/utils/timer.hpp"
#include "tsid/tasks/task-self-collision.hpp"
#include "inria_wbc/behaviors/generic/cartesian_sequential.hpp"
#include "inria_wbc/utils/ViveTracking.hpp"
#include "inria_wbc/behaviors/humanoid/follow_trackers.hpp"

#include <boost/program_options.hpp> // Boost need to be always included after pinocchio & inria_wbc

#include <typeinfo>


//defining used functions
void initialize_vive(inria::ViveTracking& vive,std::map<std::string,std::string>& vive_names);

void draw_ref(const Eigen::Vector3d& center,const Eigen::Matrix3d& rotation,
            const Eigen::Vector4d& color,std::vector<std::shared_ptr<robot_dart::Robot>>& s_list);

void update_ref(const Eigen::Vector3d& center,const Eigen::Matrix3d& rotation,
                std::vector<std::shared_ptr<robot_dart::Robot>>& s_list);

Eigen::Vector3d MatrixToEulerIntrinsic(const Eigen::Matrix3d& mat);

Eigen::Matrix3d get_trans(const std::shared_ptr<inria_wbc::behaviors::humanoid::FollowTrackers>& behavior,
                            const inria::ViveTracking& vive,const std::string vive_controller);

Eigen::Matrix3d get_rot_ref(const inria::ViveTracking& vive,const std::string vive_controller);

Eigen::Vector3d get_init_offset(const Eigen::Vector3d hand_init_pos,
                                const inria::ViveTracking& vive,const std::string vive_controller,
                                const Eigen::Matrix3d K);

Eigen::Vector3d vive_pos_processing(const inria::ViveTracking& vive,const std::string vive_controller,const Eigen::Vector3d init_offset,
                                    const Eigen::Matrix3d rot_ref,const Eigen::Matrix3d K);

Eigen::Matrix3d vive_rot_processing(const inria::ViveTracking& vive,const std::string vive_controller,
                                    const Eigen::Matrix3d trans);

void draw_obj(std::vector<std::shared_ptr<robot_dart::Robot>>& s_obj_list,
                const std::vector<std::vector<Eigen::Vector3d>>& exercises,const int index);

void update_obj(std::vector<std::shared_ptr<robot_dart::Robot>>& s_obj_list,
                const std::vector<std::vector<Eigen::Vector3d>>& exercises,const int index);

bool is_obj_achieved(Eigen::Vector3d obj,Eigen::Vector3d current_pos,double epsilon);

std::vector<std::shared_ptr<robot_dart::Robot>> draw_dist_between_two_pos(const Eigen::Vector3d pos1,
                                                                    const Eigen::Vector3d pos2,const int nb_pts);

void update_dist_between_two_pos(Eigen::Vector3d pos1,Eigen::Vector3d pos2,
                                std::vector<std::shared_ptr<robot_dart::Robot>>& s_to_cov_dist);

double talos_scaled_tracking(int argc,char* argv[],const Eigen::Matrix3d K,const double s,const double v_sat,const double thr);

double lookup(const double vin,const double& s,const double& v_sat,const double& thr);

double note(double time,int numb_of_penalties,double dist_error,int numb_of_tasks,double max_dur);
#endif