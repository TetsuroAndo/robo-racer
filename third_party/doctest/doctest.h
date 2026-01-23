#pragma once

#include <array>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace doctest {

using TestFunc = std::function< void() >;

inline std::vector< std::pair< std::string, TestFunc > > &test_registry() {
    static std::vector< std::pair< std::string, TestFunc > > registry;
    return registry;
}

inline int &fail_count() {
    static int count = 0;
    return count;
}

inline bool registerTest(const char *name, TestFunc fn) {
    test_registry().emplace_back(name, std::move(fn));
    return true;
}

inline void fail(const char *expr, const char *file, int line) {
    ++fail_count();
    std::cerr << "[doctest] CHECK failed: " << expr << " (" << file << ":" << line << ")\n";
}

inline int runAll() {
    const auto &tests = test_registry();
    for (const auto &entry : tests) {
        try {
            entry.second();
        } catch (const std::exception &e) {
            std::cerr << "[doctest] exception in test '" << entry.first << "': " << e.what() << "\n";
            ++fail_count();
        } catch (...) {
            std::cerr << "[doctest] unknown exception in test '" << entry.first << "'\n";
            ++fail_count();
        }
    }
    return fail_count();
}

} // namespace doctest

#ifdef DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
int main() {
    return doctest::runAll();
}
#endif

#define DOCTEST_CAT_IMPL(a, b) a##b
#define DOCTEST_CAT(a, b) DOCTEST_CAT_IMPL(a, b)

#define TEST_CASE(name)                                                    \
    static void DOCTEST_CAT(doctest_test_, __LINE__)();                    \
    static bool DOCTEST_CAT(doctest_register_, __LINE__) =                \
        doctest::registerTest(name, DOCTEST_CAT(doctest_test_, __LINE__)); \
    static void DOCTEST_CAT(doctest_test_, __LINE__)()

#define CHECK(expr)                                                        \
    do {                                                                   \
        if (!(expr)) {                                                     \
            doctest::fail(#expr, __FILE__, __LINE__);                     \
        }                                                                  \
    } while (0)
