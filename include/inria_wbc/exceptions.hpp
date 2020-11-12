#ifndef IWBC_EXCEPTION_HPP
#define IWBC_EXCEPTION_HPP

#include <exception>
#include <string>

// usage: throw IWBC_EXCEPTION("error:", 42)
#define IWBC_EXCEPTION(...) inria_wbc::Exception(__FILE__, __LINE__, __VA_ARGS__)

// usage: assert(x < 3, "we received x=", x)
#define IWBC_ASSERT(expr, ...)                        \
    {                                                 \
        if (!(expr))                                  \
            throw IWBC_EXCEPTION(#expr, __VA_ARGS__); \
    }

namespace inria_wbc {
    class Exception : public std::runtime_error {
    public:
        template <class... Types>
        Exception(const char* file, int line, Types... args)
            : std::runtime_error(std::string("inria_wbc:: ")
                + _make_msg(args...)
                + "\t[" + file + ":" + std::to_string(line) + "]")
        {
        }

    protected:
        // recursive variadic template unfolding
        template <typename T>
        std::string _make_msg(const T& v) const
        {
            return std::to_string(v);
        }
        template <typename T, typename... Targs>
        std::string _make_msg(const T& m, Targs... Fargs) const
        {
            return std::to_string(m) + _make_msg(Fargs...);
        }
        // special case because there is no std::to_string(const char*)...
        template <typename... Targs>
        std::string _make_msg(const char* m, Targs... Fargs) const
        {
            return std::string(m) + _make_msg(Fargs...);
        }
    };
} // namespace inria_wbc
#endif
