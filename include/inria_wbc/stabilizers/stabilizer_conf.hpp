#ifndef IWBC_STABILIZER_CONF_HPP
#define IWBC_STABILIZER_CONF_HPP

#include "inria_wbc/exceptions.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

namespace inria_wbc {
    namespace stabilizer {

        //Optional struct to gather stabilizers configuration
        struct StabConfig {
            int filter_size;
            Eigen::VectorXd com_gains;
            Eigen::VectorXd ankle_gains;
            Eigen::VectorXd ffda_gains;
            bool use_zmp;
            Eigen::VectorXd zmp_w;
            Eigen::VectorXd zmp_d;
            Eigen::VectorXd zmp_p;
            bool use_momentum;
            Eigen::VectorXd momentum_p;
            Eigen::VectorXd momentum_d;
        };

        inline std::ostream& operator<<(std::ostream& os, const StabConfig& s)
        {
            os << "StabConfig" << std::endl;
            os << "filter_size: " << s.filter_size << std::endl;
            os << "com_gains: " << s.com_gains.transpose() << std::endl;
            os << "ankle_gains: " << s.ankle_gains.transpose() << std::endl;
            os << "ffda_gains: " << s.ffda_gains.transpose() << std::endl;
            os << "use_zmp: " << s.use_zmp << std::endl;
            os << "zmp_w: " << s.zmp_w.transpose() << std::endl;
            os << "zmp_d: " << s.zmp_d.transpose() << std::endl;
            os << "zmp_p: " << s.zmp_p.transpose() << std::endl;
            os << "use_momentum: " << s.use_momentum << std::endl;
            os << "momentum_p: " << s.momentum_p.transpose() << std::endl;
            os << "momentum_d: " << s.momentum_d.transpose() << std::endl;
            return os;
        }

        inline StabConfig parse_stab_conf(const std::string& stab_path)
        {

            StabConfig sconf;
            YAML::Node s = IWBC_CHECK(YAML::LoadFile(stab_path));

            sconf.com_gains.resize(6);
            sconf.ankle_gains.resize(6);
            sconf.ffda_gains.resize(3);
            sconf.zmp_p.resize(6);
            sconf.zmp_d.resize(6);
            sconf.zmp_w.resize(6);
            sconf.momentum_p.resize(6);
            sconf.momentum_d.resize(6);

            sconf.com_gains.setZero();
            sconf.ankle_gains.setZero();
            sconf.ffda_gains.setZero();
            sconf.zmp_p.setZero();
            sconf.zmp_d.setZero();
            sconf.zmp_w.setZero();
            sconf.momentum_p.setZero();
            sconf.momentum_d.setZero();

            IWBC_ASSERT(IWBC_CHECK(s["com"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p for the com stabilizer");
            IWBC_ASSERT(IWBC_CHECK(s["ankle"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in d for the ankle stabilizer");
            IWBC_ASSERT(IWBC_CHECK(s["ffda"].as<std::vector<double>>()).size() == 3, "you need 6 coefficient in p for the ffda stabilizer");
            IWBC_ASSERT(IWBC_CHECK(s["zmp_p"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p for the zmp stabilizer");
            IWBC_ASSERT(IWBC_CHECK(s["zmp_d"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in d for the zmp stabilizer");
            IWBC_ASSERT(IWBC_CHECK(s["zmp_w"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in w for the zmp stabilizer");
            IWBC_ASSERT(IWBC_CHECK(s["momentum_p"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p for the momentum stabilizer");
            IWBC_ASSERT(IWBC_CHECK(s["momentum_d"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in d for the momentum stabilizer");

            sconf.com_gains = Eigen::VectorXd::Map(IWBC_CHECK(s["com"].as<std::vector<double>>()).data(), sconf.com_gains.size());
            sconf.ankle_gains = Eigen::VectorXd::Map(IWBC_CHECK(s["ankle"].as<std::vector<double>>()).data(), sconf.ankle_gains.size());
            sconf.ffda_gains = Eigen::VectorXd::Map(IWBC_CHECK(s["ffda"].as<std::vector<double>>()).data(), sconf.ffda_gains.size());
            sconf.zmp_p = Eigen::VectorXd::Map(IWBC_CHECK(s["zmp_p"].as<std::vector<double>>()).data(), sconf.zmp_p.size());
            sconf.zmp_d = Eigen::VectorXd::Map(IWBC_CHECK(s["zmp_d"].as<std::vector<double>>()).data(), sconf.zmp_d.size());
            sconf.zmp_w = Eigen::VectorXd::Map(IWBC_CHECK(s["zmp_w"].as<std::vector<double>>()).data(), sconf.zmp_w.size());
            sconf.momentum_p = Eigen::VectorXd::Map(IWBC_CHECK(s["momentum_p"].as<std::vector<double>>()).data(), sconf.momentum_p.size());
            sconf.momentum_d = Eigen::VectorXd::Map(IWBC_CHECK(s["momentum_d"].as<std::vector<double>>()).data(), sconf.momentum_d.size());

            sconf.use_zmp = IWBC_CHECK(s["use_zmp"].as<bool>());
            sconf.use_momentum = IWBC_CHECK(s["use_momentum"].as<bool>());
            sconf.filter_size = IWBC_CHECK(s["filter_size"].as<int>());

            return sconf;
        }

    } // namespace stabilizer
} // namespace inria_wbc
#endif