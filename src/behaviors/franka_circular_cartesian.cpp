#include "inria_wbc/behaviors/franka_circular_cartesian.hpp"

namespace inria_wbc {
    namespace behaviors {

        static Register<CircCartTraj> __franka_circular_cartesian_trajectory("circular-cartesian-trajectory");

        CircCartTraj::CircCartTraj(const controller_ptr_t& controller) : Behavior(controller)
        {
            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            dt_ = controller_->dt();


            _generate_trajectories();
        }

        void CircCartTraj::_generate_trajectories(){//to do
        }
        

        void CircCartTraj::update(const controllers::SensorData& sensor_data)
        {//~~ to do

        }

    } // namespace behaviors
} // namespace inria_wbc
