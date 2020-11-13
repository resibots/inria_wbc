#ifndef TEST_BEHAVIOR_HPP_
#define TEST_BEHAVIOR_HPP_

#include <map>
#include <vector>

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> inline test_behavior(
    const std::shared_ptr<inria_wbc::behaviors::Behavior>& behavior)
{
    std::cout << "running...";
    std::cout.flush();
    auto controller = behavior->controller();

    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    uint ncontrollable = controllable_dofs.size();

    std::vector<Eigen::VectorXd> cmds, cmds_filtered;
    inria_wbc::controllers::SensorData sensors;
    for (int i = 0; i < 100; ++i) {
        std::cout << i << " ";
        std::cout.flush();
        behavior->update(sensors); // default values for sensors
        auto cmd = controller->q(false);
        cmds.push_back(cmd);
        cmds_filtered.push_back(controller->filter_cmd(cmd).tail(ncontrollable));
    }
    std::cout << "done" << std::endl;
    return std::make_pair(cmds, cmds_filtered);
}

template <typename T>
inline void compare_cmds(const T& cmds, const T& cmds2)
{
    std::cout << "Comparing commands:";
    BOOST_CHECK_EQUAL(cmds.first.size(), cmds2.first.size());
    for (int i = 0; i < cmds.first.size(); ++i) {
        std::cout << i << " ";
        std::cout.flush();
        BOOST_CHECK(cmds.first[i].isApprox(cmds2.first[i], 1e-8));
        BOOST_CHECK(cmds.second[i].isApprox(cmds2.second[i], 1e-8));
    }
    std::cout << "done" << std::endl;
}

#endif
