#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>

namespace inria_wbc {
    namespace utils {
        class Timer {
        public:
            void begin(const std::string& name)
            {
                _start[name] = std::chrono::high_resolution_clock::now();
            }
            void end(const std::string& name)
            {
                auto e = std::chrono::high_resolution_clock::now();
                double t = std::chrono::duration_cast<std::chrono::microseconds>(e - _start[name]).count();
                auto it = _data.find(name);
                if (it == _data.end())
                    _data[name] = {0, t, t, t};
                else {
                    it->second.iterations += 1;
                    it->second.time += t;
                    it->second.min_time = std::min(t, it->second.min_time);
                    it->second.max_time = std::max(t, it->second.max_time);
                }
            }
            // with period = -1, we do not update the counter (to report in several streams)
            void report(double t, int period = 100)
            {
                report(std::cout, t, period, '\t');
            }
            // with period = -1, we do not update the counter (to report in several streams)
            void report(std::ostream& oss, double t, int period = 100, char separator = '\t')
            {
                if (period != -1)
                    _k++;
                if (period != -1 && _k != period + 1)
                    return;
                oss << "t:" << t << separator;
                for (auto& x : _data) { // display in ms
                    oss.precision(3);
                    oss << x.first
                        << ":" << (x.second.time / x.second.iterations) / 1000.0 << "ms"
                        << " [" << x.second.min_time / 1000.0 << ":" << x.second.max_time / 1000.0 << "]"
                        << separator;
                }
                oss << std::endl;
                _k = 1;
                _data.clear();
            }
            int iteration() const { return _k; }

        protected:
            int _k = 1;
            using time_t = std::chrono::high_resolution_clock::time_point;
            struct info_t {
                int iterations;
                double time;
                double min_time;
                double max_time;
            };
            std::unordered_map<std::string, time_t> _start;
            std::unordered_map<std::string, info_t> _data;
        };
    } // namespace utils
} // namespace inria_wbc