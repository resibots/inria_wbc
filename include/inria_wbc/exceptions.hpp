#ifndef IWBC_EXCEPTION_HPP
#define IWBC_EXCEPTION_HPP

#include <exception>
#include <string>

#if IWBC_USE_STACKTRACE
#include <boost/stacktrace.hpp> // require boost 1.65+
#endif



// TODO : use __PRETTY_FUNCTION__ or __FUNCTION__

// usage: throw IWBC_EXCEPTION("error:", 42)
#define IWBC_EXCEPTION(...) inria_wbc::Exception(__FILE__, __LINE__, __VA_ARGS__)
#define IWBC_FIRST_ARG(N, ...) N
#define IWBC_FIRST_ARG_STR(N, ...) #N
#define IWBC_NEXT_ARG_SUB(ignore, ...) __VA_ARGS__
#define IWBC_NEXT_ARG(...) IWBC_NEXT_ARG_SUB(__VA_ARGS__)

// some magic to add the line numbers / function / call stack to an existing exception (e.g. YAML)
// usage: IWBC_CHECK(function_that_can_throw());
// (we create a lambda and call it directly)
#define IWBC_CHECK(T) [&] {                                     \
    try { return T;}                                            \
    catch (std::runtime_error& e) {                                 \
           throw IWBC_EXCEPTION("[", e.what(), "] when calling: ", std::string(std::string(#T)));                      \
    }}()

// usage: IWBC_ASSERT(x < 3, "we received x=", x)
#define IWBC_ASSERT(...)                                                             \
    {                                                                                \
        if (!(IWBC_FIRST_ARG(__VA_ARGS__)))                                          \
            throw IWBC_EXCEPTION(IWBC_FIRST_ARG_STR(__VA_ARGS__), " ", IWBC_NEXT_ARG(__VA_ARGS__)); \
    }

#define IWBC_ERROR(...)                    \
    {                                      \
        throw IWBC_EXCEPTION(__VA_ARGS__); \
    }

namespace inria_wbc {

#ifdef IWBC_USE_STACKTRACE
    template<typename T>
    inline std::string stacktrace_to_string(const T& bt) {
        return boost::stacktrace::detail::to_string(&bt[0], bt.size());
    }
#endif

    class Exception : public std::runtime_error {
    public:
        template <class... Types>
        Exception(const char* file, int line, Types... args)
            : std::runtime_error(std::string("inria_wbc:: ")
                + _make_msg(args...)
                + "\t[" + file + ":" + std::to_string(line) + "]\n"
                + "\n------ stack ------\n"
#ifdef IWBC_USE_STACKTRACE
                + stacktrace_to_string(boost::stacktrace::stacktrace())
#endif
                )
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
        template <typename... Targs>
        std::string _make_msg(const std::string& s, Targs... Fargs) const
        {
            return s + _make_msg(Fargs...);
        }
        std::string _make_msg() const
        {
            return std::string("");
        }
    };
} // namespace inria_wbc
#endif
