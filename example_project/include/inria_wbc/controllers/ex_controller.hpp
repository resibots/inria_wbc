#ifndef IWBC_EX_CONTROLLER_HPP
#define IWBC_EX_CONTROLLER_HPP

#include <inria_wbc/controllers/talos_pos_tracking.hpp>

namespace inria_wbc {
    namespace controllers {

        class ExController : public TalosPosTracking {
        public:
            ExController(const Params& params);
            ExController(const ExController& other) = delete;
            ExController& operator=(const ExController& o) const = delete;
            virtual ~ExController(){};

            virtual const opt_params_t& opt_params() const override { return params_.opt_params; }

        protected:
            virtual void parse_configuration_yaml(const std::string& sot_config_path) override;
            virtual void set_stack_configuration() override;
            virtual void set_default_opt_params(std::map<std::string, double>& p);
        };

    } // namespace controllers
} // namespace inria_wbc
#endif
