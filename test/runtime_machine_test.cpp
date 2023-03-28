#include "mock_common.h"

#define EQ_R(classname, name, ...)                           \
    TEST_F(MockInterfaceTest, name)                          \
    {                                                        \
        EXPECT_EQ(cli->get##classname()->name(__VA_ARGS__),  \
                  ReturnValue<function_traits<decltype(      \
                      &classname::name)>::return_type>()()); \
    }

#define EQ_R0(classname, name)                               \
    TEST_F(MockInterfaceTest, name)                          \
    {                                                        \
        EXPECT_EQ(cli->get##classname()->name(),             \
                  ReturnValue<function_traits<decltype(      \
                      &classname::name)>::return_type>()()); \
    }

#define EQ_R1(classname, name)                                     \
    TEST_F(MockInterfaceTest, name)                                \
    {                                                              \
        EXPECT_EQ(cli->get##classname()->name(                     \
                      ParamValue<function_traits<decltype(         \
                          &classname::name)>::args<0>::type>()()), \
                  ReturnValue<function_traits<decltype(            \
                      &classname::name)>::return_type>()());       \
    }

#define EQ_R3(classname, name)                                     \
    TEST_F(MockInterfaceTest, name)                                \
    {                                                              \
        EXPECT_EQ(cli->get##classname()->name(                     \
                      ParamValue<function_traits<decltype(         \
                          &classname::name)>::args<0>::type>()(),  \
                      ParamValue<function_traits<decltype(         \
                          &classname::name)>::args<1>::type>()(),  \
                      ParamValue<function_traits<decltype(         \
                          &classname::name)>::args<2>::type>()()), \
                  ReturnValue<function_traits<decltype(            \
                      &classname::name)>::return_type>()());       \
    }

#define _FUNC(m, n, method, ...) EQ_R##n(m, method)
#define _INST(m, n, method, ...) EQ_R##n(m, method)

RuntimeMachine_DECLARES
