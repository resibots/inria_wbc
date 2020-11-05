#ifndef IWBC_UTILS_FACTORY_HPP
#define IWBC_UTILS_FACTORY_HPP

namespace inria_wbc {
    namespace utils {

        // a generic factory class, to be specialized for your own class
        // first is the objects to create (abstract class), second is the parameter types for the constructor
        template <typename T, typename A>
        class Factory {
        public:
            ~Factory() { _map.clear(); }

            static Factory& instance()
            {
                static Factory instance;
                return instance;
            }
            using ptr_t = std::shared_ptr<T>;
            using creator_t = std::function<ptr_t(const A&)>;

            void register_creator(const std::string& name, creator_t pfn_creator)
            {
                if (_map.find(name) == _map.end())
                    _map[name] = pfn_creator;
                else {
                    std::cout << "Warning : there is already a " << name << " in the factory" << std::endl;
                }
            }
            ptr_t create(const std::string& name, const A& params)
            {
                auto it = _map.find(name);
                if (it != _map.end())
                    return it->second(params);
                else
                    std::cerr << "Error :  " << name << " is not in the factory" << std::endl;
                assert(0);
                return ptr_t();
            }

            void print()
            {
                for (auto& it : _map) {
                    std::cout << it.first << std::endl;
                }
            }

        private:
            Factory() {}
            Factory& operator=(const Factory&) { return *this; }
            std::map<std::string, creator_t> _map;
        };

        template <typename B, typename T, typename C>
        struct AutoRegister {
            AutoRegister(const std::string& name)
            {
                Factory<B, C>::instance().register_creator(name, [](const C& arg) {
                    return std::make_shared<T>(arg);
                });
            }
        };
    } // namespace utils
} // namespace inria_wbc

#endif