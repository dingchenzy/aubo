#ifndef MOCK_COMMON_H
#define MOCK_COMMON_H

#include <gtest/gtest.h>

#include <jsonrpccxx/client.hpp>
#include "aubo/type_def.h"
#include "bindings/jsonrpc/jsonrpc_service.hpp"
#include "bindings/jsonrpc/jsonrpc_client.hpp"
#include "inmemoryconnector.hpp"
#include "bindings/function_traits.h"

using namespace arcs::common_interface;
using namespace jsonrpccxx;
using namespace testing;
using ::testing::Matcher;
using ::testing::MatchesRegex;

class MockInterfaceTest : public testing::Test
{
protected:
    static void SetUpTestSuite();
    static void TearDownTestSuite();

    static std::shared_ptr<AuboApi> interface;
    static std::shared_ptr<JsonRpcService> service;
    static std::shared_ptr<InMemoryConnector> connector;
    static std::shared_ptr<AuboApi> cli;
};

// clang-format off
// 宏定义自动化mock test原理(使用template,根据取到参数类型来指定调用对应返回值类型的数值)(https://www.cnblogs.com/qicosmos/p/4772328.html)
// define宏参数注释: ##字符串拼接 #取变量符串 (https://www.jianshu.com/p/cfac34b4a0f7)
// clang-format on

#define EQ(classname, name, ...)                                           \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(__VA_ARGS__),              \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ0(classname, name)                                               \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(),                         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ1(classname, name)                                               \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(                           \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<0>::type>()()),         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ2(classname, name)                                               \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(                           \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<0>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<1>::type>()()),         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ3(classname, name)                                               \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(                           \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<0>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<1>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<2>::type>()()),         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ4(classname, name)                                               \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(                           \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<0>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<1>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<2>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<3>::type>()()),         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ5(classname, name)                                               \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(                           \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<0>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<1>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<2>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<3>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<4>::type>()()),         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ6(classname, name)                                               \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(                           \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<0>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<1>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<2>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<3>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<4>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<5>::type>()()),         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ9(classname, name)                                               \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(                           \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<0>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<1>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<2>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<3>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<4>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<5>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<6>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<7>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<8>::type>()()),         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

#define EQ10(classname, name)                                              \
    TEST_F(MockInterfaceTest, name)                                        \
    {                                                                      \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front()); \
        EXPECT_EQ(robot->get##classname()->name(                           \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<0>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<1>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<2>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<3>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<4>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<5>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<6>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<7>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<8>::type>()(),          \
                      ParamValue<function_traits<decltype(                 \
                          &classname::name)>::args<9>::type>()()),         \
                  ReturnValue<function_traits<decltype(                    \
                      &classname::name)>::return_type>()());               \
    }

// 返回值3个vector<double>
#define NEAR3(classname, name, ...)                                            \
    TEST_F(MockInterfaceTest, name)                                            \
    {                                                                          \
        auto robot = cli->getRobotInterface(cli->getRobotNames().front());     \
        auto tmp = ReturnValue<                                                \
            function_traits<decltype(&classname::name)>::return_type>()();     \
        for (int i = 0; i < 6; i++) {                                          \
            EXPECT_NEAR(                                                       \
                std::get<0>(robot->get##classname()->name(__VA_ARGS__)).at(i), \
                std::get<0>(tmp).at(i), 0.1);                                  \
            EXPECT_NEAR(                                                       \
                std::get<1>(robot->get##classname()->name(__VA_ARGS__)).at(i), \
                std::get<1>(tmp).at(i), 0.1);                                  \
            EXPECT_NEAR(                                                       \
                std::get<2>(robot->get##classname()->name(__VA_ARGS__)).at(i), \
                std::get<2>(tmp).at(i), 0.1);                                  \
        }                                                                      \
    }

#endif
