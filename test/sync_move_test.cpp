#include "mock_common.h"
#define EQ_S(classname, name, ...)                           \
    TEST_F(MockInterfaceTest, name)                          \
    {                                                        \
        EXPECT_EQ(cli->get##classname()->name(__VA_ARGS__),  \
                  ReturnValue<function_traits<decltype(      \
                      &classname::name)>::return_type>()()); \
    }

#define EQ_S0(classname, name)                               \
    TEST_F(MockInterfaceTest, name)                          \
    {                                                        \
        EXPECT_EQ(cli->get##classname()->name(),             \
                  ReturnValue<function_traits<decltype(      \
                      &classname::name)>::return_type>()()); \
    }

#define EQ_S1(classname, name)                                     \
    TEST_F(MockInterfaceTest, name)                                \
    {                                                              \
        EXPECT_EQ(cli->get##classname()->name(                     \
                      ParamValue<function_traits<decltype(         \
                          &classname::name)>::args<0>::type>()()), \
                  ReturnValue<function_traits<decltype(            \
                      &classname::name)>::return_type>()());       \
    }

#define EQ_S3(classname, name)                                     \
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

#define _FUNC(m, n, method, ...) EQ_S##n(m, method)
#define _INST(m, n, method, ...) EQ_S##n(m, method)
SystemInfo_DECLARES
