#ifndef IWBC_UTILS_FACTORY_HPP
#define IWBC_UTILS_FACTORY_HPP

#include <inria_wbc/exceptions.hpp>

namespace inria_wbc {
    namespace utils {

        // a generic factory class, to be specialized for your own class
        // first is the objects to create (abstract class), second is the parameter types for the constructor
        template <typename T, class... Types>
        class Factory {
        public:
            template <typename B>
            struct AutoRegister {
                AutoRegister(const std::string& name)
                {
                    instance().register_creator(name, [](const Types&... args) {
                        return std::make_shared<B>(args...);
                    });
                }
            };

            static Factory& instance()
            {
                static Factory instance;
                return instance;
            }
            using ptr_t = std::shared_ptr<T>;
            using creator_t = std::function<ptr_t(const Types&... args)>;
            
            void register_creator(const std::string& name, const creator_t& pfn_creator)
            {
                if (_map.find(name) == _map.end())
                    _map[name] = pfn_creator;
                else {
                    std::cout << "Warning : there is already a " << name << " in the factory" << std::endl;
                }
            }
            ptr_t create(const std::string& name, const Types&... args)
            {
                auto it = _map.find(name);
                if (it != _map.end())
                    return it->second(args...);
                else
                    throw IWBC_EXCEPTION(name, " is not in the factory");
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
            std::unordered_map<std::string, creator_t> _map;
        };
    }; // namespace utils
} // namespace inria_wbc

#endif