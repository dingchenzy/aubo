#include "mock_common.h"
#include "bindings/impl/pimpl.h"

// 静态成员变量定义
std::shared_ptr<AuboApi> MockInterfaceTest::interface;
std::shared_ptr<JsonRpcService> MockInterfaceTest::service;
std::shared_ptr<InMemoryConnector> MockInterfaceTest::connector;
std::shared_ptr<AuboApi> MockInterfaceTest::cli;

namespace arcs {
namespace common_interface {
using namespace jsonrpc_client;

#define _FUNC(m, n, f, ...) IMPL##n(m, f, __VA_ARGS__)
#define _INST(m, n, f, ...) IMPL##n(m, f, __VA_ARGS__)
IMPL_ALL
#undef _FUNC
#undef _INST

} // namespace common_interface
} // namespace arcs

class AuboApi1 : public AuboApi
{
public:
    AuboApi1(std::shared_ptr<IClientConnector> connector, bool fake)
    {
        d_ = new AuboApiImpl(connector, fake);
    }
    ~AuboApi1() { delete (AuboApiImpl *)d_; }

    AuboApiImpl *getImpl() { return (AuboApiImpl *)d_; }
};

void MockInterfaceTest::SetUpTestSuite()
{
    // mock类,先通过MOCK_METHOD0函数声明mock函数，再通过EXPECT_CALL来指定返回值
    interface = std::make_shared<AuboApi1>(nullptr, true);

    // Rpc服务端
    service = std::make_shared<JsonRpcService>(interface);

    // Rpc服务端和客户端链接件
    connector = std::make_shared<InMemoryConnector>(*service);

    // Rpc客户端
    cli = std::make_shared<AuboApi1>(connector, false);
    std::dynamic_pointer_cast<AuboApi1>(cli)->getImpl()->updateRobotNames();
}

void MockInterfaceTest::TearDownTestSuite()
{
    interface.reset();
    service.reset();
    connector.reset();
    cli.reset();
}
