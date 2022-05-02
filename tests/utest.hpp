
#ifndef UTEST_HPP__
#define UTEST_HPP__

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace utest {

    namespace colors {
        static const std::string red = "\x1B[31m";
        static const std::string green = "\x1B[32m";
        static const std::string yellow = "\x1B[33m";
        static const std::string blue = "\x1B[34m";
        static const std::string magenta = "\x1B[35m";
        static const std::string cyan = "\x1B[36m";
        static const std::string grey = "\x1B[90m";
        static const std::string rst = "\x1B[0m";
        static const std::string bold = "\x1B[1m";
    } // namespace colors
    namespace res {
        enum result_t { error = 0,
            ok = 1,
            warning = 2,
            info = 3 };
    }
    struct TestResult {
        std::string test;
        std::string msg;
        std::string file;
        int line;
        res::result_t result;
    };

    struct Test {
        using function_t = std::function<void(void)>;
        Test(const std::string& name_) : name(name_) {}
        res::result_t ok() const
        {
            int warning = 0;
            for (auto& x : test_results) {
                if (x.result == res::error)
                    return res::error;
                warning += (x.result == res::warning) ? 1 : 0;
            }
            if (warning != 0)
                return res::warning;
            return res::ok;
        }
        //
        std::string name;
        function_t function;
        std::vector<TestResult> test_results;
    };
    using test_t = std::shared_ptr<Test>;
    test_t make_test(const std::string& name) { return std::make_shared<Test>(name); }

    struct TestSuite {
        void run(int n_threads = std::thread::hardware_concurrency(), bool verbose = true)
        {
            if (n_threads == -1)
                n_threads = std::thread::hardware_concurrency();
            if (test_cases.size() == 0) {
                std::cerr << "utest::Warning: empty test suite!" << std::endl;
                return;
            }
            n_threads = std::min(n_threads, int(test_cases.size()));
            std::vector<std::shared_ptr<std::thread>> threads(n_threads);
            
            int cases_per_thread = std::max(int(std::ceil((float)test_cases.size() / n_threads)), 1);
            if (verbose) {
                std::cout << "test cases:" << test_cases.size() << std::endl;
                std::cout << "nthreads:" << n_threads << " # max cases/thread:" << cases_per_thread << std::endl;
            }
            for (int t = 0; t < n_threads; ++t) {
                threads[t] = std::make_shared<std::thread>([=] {
                    for (int i = 0; i < cases_per_thread; ++i) {
                        unsigned int c = t + i * n_threads;
                        if(c >= test_cases.size())
                            break;
                        if (verbose) {
                            std::cout << ".";
                            std::cout.flush();
                        }
                        test_cases[c]->function();
                } });
                //threads[t]->join();
            }
            for (auto& t : threads)
                t->join();
            if (verbose)
                std::cout << std::endl;
        }
        void reg(const std::shared_ptr<Test>& test, const Test::function_t& function)
        {
            test->function = function;
            test_cases.push_back(test);
        }
        bool success() const
        {
            for (auto& t : test_cases) {
                if (t->ok() == res::error)
                    return false;
            }
            return true;
        }
        std::vector<test_t> test_cases;
    };
    // this is not in the class to make it easy to write you own reporting function
    void write_report(const TestSuite& test_suite, std::ostream& os, bool full = true)
    {
        double total = 0;
        // for each test
        for (auto& t : test_suite.test_cases) {
            total += t->ok() ? 1 : 0;
            if (t->ok() == res::ok)
                os << colors::green << "OK " << colors::rst << " => " << t->name << std::endl;
            else if (t->ok() == res::warning)
                os << colors::yellow << "OK (W)" << colors::rst << " => " << t->name << std::endl;
            else
                os << colors::red << "ERROR" << colors::rst << " => " << t->name << std::endl;

            // for each check in the test
            for (auto& x : t->test_results)
                if (x.result == res::error || full) {
                    if (x.result == res::ok && full)
                        os << colors::green << "\tOK ";
                    else if (x.result == res::info && full)
                        os << colors::green << "\tINFO ";
                    else if (x.result == res::warning)
                        os << colors::yellow << "\tWARNING ";
                    else if (x.result == res::error)
                        os << colors::red << "\tERROR ";

                    if (x.result == res::error || full)
                        os << colors::rst << x.msg << " [" << x.test << "] " << colors::grey << " (" << x.file << ":" << x.line << ")" << colors::rst << std::endl;
                }
            if (full)
                os << std::endl;
        }
        os << "=> success: " << total / test_suite.test_cases.size() * 100.0 << "%" << std::endl;
    }
    std::string strip_path(const std::string& s)
    {
        return s.substr(s.find_last_of("/") + 1);
    }
}; // namespace utest

#define __UTEST_FILE utest::strip_path(__FILE__)

#define __UTEST_CHECK_MESSAGE(test, message, to_test, level)                                            \
    {                                                                                                   \
        utest::res::result_t ok = utest::res::ok;                                                       \
        try {                                                                                           \
            ok = (bool)(to_test) ? utest::res::ok : level;                                              \
        }                                                                                               \
        catch (std::exception) {                                                                        \
            ok = level;                                                                                 \
        }                                                                                               \
        test->test_results.push_back(utest::TestResult{#to_test, message, __UTEST_FILE, __LINE__, ok}); \
    }

#define UTEST_CHECK_MESSAGE(test, message, to_test) __UTEST_CHECK_MESSAGE(test, message, to_test, utest::res::error)
#define UTEST_CHECK(test, to_test) UTEST_CHECK_MESSAGE(test, "", to_test)
#define UTEST_WARN(test, to_test) __UTEST_CHECK_MESSAGE(test, "", to_test, utest::res::warning)
#define UTEST_WARN_MESSAGE(test, msg, to_test) __UTEST_CHECK_MESSAGE(test, msg, to_test, utest::res::warning)

#define UTEST_CHECK_EXCEPTION(test, to_test)                                                       \
    {                                                                                              \
        utest::res::result_t ok = utest::res::error;                                               \
        try {                                                                                      \
            to_test;                                                                               \
        }                                                                                          \
        catch (std::exception) {                                                                   \
            ok = utest::res::ok;                                                                   \
        }                                                                                          \
        test->test_results.push_back(utest::TestResult{#to_test, "", __UTEST_FILE, __LINE__, ok}); \
    }

#define UTEST_CHECK_NO_EXCEPTION(test, to_test)                                                    \
    {                                                                                              \
        utest::res::result_t ok = utest::res::ok;                                                  \
        try {                                                                                      \
            to_test;                                                                               \
        }                                                                                          \
        catch (std::exception) {                                                                   \
            ok = utest::res::error;                                                                \
        }                                                                                          \
        test->test_results.push_back(utest::TestResult{#to_test, "", __UTEST_FILE, __LINE__, ok}); \
    }

#define UTEST_ERROR(test, msg)                                                                               \
    {                                                                                                        \
        test->test_results.push_back(utest::TestResult{"", msg, __UTEST_FILE, __LINE__, utest::res::error}); \
    }

#define UTEST_WARNING(test, msg)                                                                               \
    {                                                                                                          \
        test->test_results.push_back(utest::TestResult{"", msg, __UTEST_FILE, __LINE__, utest::res::warning}); \
    }

#define UTEST_INFO(test, msg)                                                                               \
    {                                                                                                       \
        test->test_results.push_back(utest::TestResult{"", msg, __UTEST_FILE, __LINE__, utest::res::info}); \
    }

#define UTEST_REGISTER(test_suite, test, function) test_suite.reg(test, [=]() { function; })

#if UTEST_EXAMPLE // an example
#include "utest.hpp"
#include <iostream>

void my_test_function(utest::test_t test, int a)
{
    UTEST_CHECK(test, a != 42);
    UTEST_CHECK(test, a * 2 != 42);
    UTEST_WARNING(test, "something might be wrong");
};

bool exc()
{
    throw std::runtime_error("test"); // throws an exception
    return true;
}

void my_test_function2(utest::test_t test)
{
    UTEST_CHECK(test, exc());
};

int main(int argc, char** argv)
{
    utest::TestSuite test_suite;

    auto test1 = utest::make_test("test 1");
    UTEST_REGISTER(test_suite, test1, my_test_function(test1, 42));

    auto test2 = utest::make_test("test 2");
    UTEST_REGISTER(test_suite, test2, my_test_function(test2, 43));

    auto test3 = utest::make_test("test 3");
    UTEST_REGISTER(test_suite, test3, my_test_function(test3, 21));

    auto test4 = utest::make_test("test 4");
    UTEST_REGISTER(test_suite, test4, my_test_function2(test4));

    test_suite.run();

    utest::write_report(test_suite, std::cout, true);

    std::cout << "\n\nlight report:" << std::endl;
    utest::write_report(test_suite, std::cout, false);

    return test_suite.success();
}
#endif

#endif