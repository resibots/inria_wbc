#ifndef TALOS_FACTORY_HPP
#define TALOS_FACTORY_HPP
#include "controllers/talos_base_controller.hpp"

namespace tsid_sot
{
    namespace example
    {
        class Example
        {
        public:
            Example(){};
            Example(const tsid_sot::controllers::TalosBaseController::Params &params,
                    const std::string &sot_config_path = "",
                    const std::string &fb_joint_name = "",
                    const std::vector<std::string> &mimic_joint_names = {},
                    bool verbose = false);

            virtual Eigen::VectorXd cmd() = 0;

            virtual std::shared_ptr<tsid_sot::controllers::TalosBaseController> controller() { return controller_; };

        protected:
            std::shared_ptr<tsid_sot::controllers::TalosBaseController> controller_;
        };

        class ExampleFactory
        {
        public:
            ~ExampleFactory() { example_map_.clear(); }

            static ExampleFactory &instance()
            {
                static ExampleFactory instance;
                return instance;
            }
            typedef std::shared_ptr<Example> example_ptr_t;
            typedef std::function<example_ptr_t(const tsid_sot::controllers::TalosBaseController::Params &,
                                                const std::string &,
                                                const std::string &,
                                                const std::vector<std::string> &,
                                                bool verbose)>
                example_creator_t;

            void register_example(const std::string &example_name, example_creator_t pfn_create_example)
            {
                if (example_map_.find(example_name) == example_map_.end())
                {
                    example_map_[example_name] = pfn_create_example;
                }
                else
                {
                    std::cout << "Warning : there is already a " << example_name << " example in the factory" << std::endl;
                }
            }

            example_ptr_t create_example(const std::string &example_name,
                                         const tsid_sot::controllers::TalosBaseController::Params &params,
                                         const std::string &sot_config_path = "",
                                         const std::string &fb_joint_name = "",
                                         const std::vector<std::string> &mimic_joint_names = {},
                                         bool verbose = false)
            {
                auto it = example_map_.find(example_name);
                if (it != example_map_.end())
                    return it->second(params, sot_config_path, fb_joint_name, mimic_joint_names, verbose);
                else
                    std::cerr << "Error :  " << example_name << " is not in the example factory" << std::endl;
            }

            void controller_names()
            {
                for (auto &it : example_map_)
                {
                    std::cout << it.first << std::endl;
                }
            }

        private:
            ExampleFactory() {}
            ExampleFactory(const ExampleFactory &) {}
            ExampleFactory &operator=(const ExampleFactory &) { return *this; }
            std::map<std::string, example_creator_t> example_map_;
        };
        template <typename ExampleClass>
        struct AutoRegister
        {
            AutoRegister(std::string example_name)
            {
                ExampleFactory::instance().register_example(example_name, [](const tsid_sot::controllers::TalosBaseController::Params &params,
                                                                             const std::string &sot_config_path = "",
                                                                             const std::string &fb_joint_name = "",
                                                                             const std::vector<std::string> &mimic_joint_names = {},
                                                                             bool verbose = false) {
                    return std::make_shared<ExampleClass>(params, sot_config_path, fb_joint_name, mimic_joint_names, verbose);
                });
            }
        };

    } // namespace example
} // namespace tsid_sot
#endif