# 接口开发流程

<br/>

## 开发步骤大致路线:

**step1** 【查看,编写】对外的**接口**头文件和**数据类型**头文件(type_def.h) 

- common_interface/include/common_interface/                  

**step2**  【编写】jsonrpc->client端，对外接口的 jsonrpc 协议的**编码**功能 

- common_interface/include/jsonrpc/                                    

**step3** 【编写】jsonroc->server端，注册实现对外接口的功能函数 ,**解码**之后会去调用真正的实现函数 

- common_interface/include/jsonrpc/jsonrpc_service.hpp  

**step4** 【编写】使用gmock框架, 创建对外接口的mock类并使用宏**MOCK_METHODx**定义对应的函数,使用宏**EXPECT_CALL**指定函数的返回值                             

- common_interface/include/mock/mock_robot_interface.h                                       

**step5** 【编写】使用gmock框架, 宏**EXPECT_EQ**测试对外接口 

- common_interface/test/  

<br/>

## 暂不支持(随后会解决):

- 暂不支持传入参数为数组：kinematicsIdentify

<br/>

## 各接口类型示例

<br/>

> #### **Demo1**
>

**接口函数:** int ForceControl::fcEnable()

**示例类型:** 

- 传入参数个数为0
- 返回值为int

**step 2:**

**jsonrpccxx::JsonRpcClient::CallMethod** 参数说明: **传入参数为0可以使用CallMethod函数**, <int>代表返回值，jsonrpccxx::get_uuid()代表id，"ForceControl.fcEnable"代表类下的成员函数，中间用.隔开，{}代表传入参数

```c++
int fcEnable() override
{
	return client_.CallMethod<int>(jsonrpccxx::get_uuid(),"ForceControl.fcEnable", {});
}
```

**step 3:**

此宏函数为宏定义函数, **要注意里面的参数为本成员函数对应的类**

```c++
DEF(fcEnable, {});
```

**step 4:**

(1)对应的mock文件中定义mock函数

MOCK_METHODx宏函数说明: x代表参数个数，第一个参数是成员函数名字，第二个参数是函数原型剔除掉函数名剩下的那部分

```c++
MOCK_METHOD0(fcEnable, int());
```

(2)[mock_robot_interface.h]()中指定函数动作,也就是返回值(gmock原型宏是使用EXPECT_CALL,这里我们增加了一次封装) 

```c++
CALL0(fcEnable);
```

**step 5:**

使用gtest宏函数TEST_F,宏函数第一个参数是自定义的testing::Test派生类, 第二个参数是测试名字

函数内部使用gtest宏**EXPECT_EQ**，判断返回值是否正确

```c++
TEST_F(MockInterfaceTest, fcEnable)
{
    EXPECT_EQ(cli->getForceControl()->fcEnable(), 0);
}
```

<br/>

> #### Demo2:
>

**接口函数:** int ForceControl::setToolMassInertia(double mass, const std::vector<double> &com, const std::vector<double> &inertia)

**示例类型:** 

- 传入参数为多个(**double **,  **std::vector**)
- 返回值为int

**step 2:**

**jsonrpccxx::JsonRpcClient::CallMethodNamed** 参数说明: **传入参数为多个需要使用CallMethodNamed**函数**, 

传入参数形式:   { {"参数1的字符串"，参数1的值} , {"参数2的字符串"，参数2的值} }

```c++
int setToolMassInertia(double mass, const std::vector<double> &com,const std::vector<double> &inertia) override
{
        return client_.CallMethodNamed<int>(jsonrpccxx::get_uuid(), "ForceControl.setToolMassInertia",
            { { "mass", mass }, { "com", com }, { "inertia", inertia } });
}
```

**step 3:**

注意传入参数形式: {"参数1字符串", "参数2字符串", "参数3字符串"}

```c++
DEF(setToolMassInertia, { "mass", "com", "inertia" });
```

**step 4:**

这里参数个数是3，所以是MOCK_METHOD3. 第一个参数是成员函数名字，第二个参数是函数原型剔除掉函数名剩下的那部分

```c++
MOCK_METHOD3(setToolMassInertia, int(double, const std::vector<double> &,const std::vector<double> &));
```

**step 5:**

```
TEST_F(MockInterfaceTest, setToolMassInertia)
{
    double mass = 0.5;
    const std::vector<double> com(6, 0.3);
    const std::vector<double> inertia(6, 1.2);
    EXPECT_EQ(cli->getForceControl()->setToolMassInertia(mass, com, inertia),
              0);
}
```

<br/>

> #### Demo3:
>

**接口函数:** int IoControl::setConfigurableInputAction(uint32_t index,**SafetyInputAction** action)

**示例类型:** 

- 传入参数为多个,并且有枚举
- 返回值为int

**step 1:**

注册枚举类型, 用于枚举类型和json之间的转换，type_def.h文件中

```c++
// SafetyInputAction
NLOHMANN_JSON_SERIALIZE_ENUM(
    SafetyInputAction,
    { { SafetyInputAction::Unassigned, "Unassigned" },
      { SafetyInputAction::EmergencyStop, "EmergencyStop" },
      { SafetyInputAction::ReducedMode, "ReducedMode" },
      { SafetyInputAction::SafeguardReset, "SafeguardReset" },
      { SafetyInputAction::ThreePositionSwitch, "ThreePositionSwitch" },
      { SafetyInputAction::OperationalMode, "OperationalMode" } })
```

**step 2:**

```c++
int setConfigurableInputAction(uint32_t index, SafetyInputAction action) override
{
        return client_.CallMethodNamed<int>(jsonrpccxx::get_uuid(), "IoControl.setConfigurableInputAction",
            { { "index", index }, { "action", action } });
}
```

**step 3:**

```c++
DEF(setConfigurableInputAction, { "index", "action" });
```

**step 4:**

```c++
CALL2(setConfigurableInputAction,int(uint32_t,SafetyInputAction));
```

**step 5:**

```c++
TEST_F(MockInterfaceTest, setConfigurableInputAction)
{
    EXPECT_EQ(cli->getIoControl()->setConfigurableInputAction(
                  1, SafetyInputAction::Unassigned),
              0);
}
```

<br/>

> #### Demo4:
>

**接口函数:** **StandardOutputRunState** IoControl::getDigitalOutputRunstate(uint32_t index);

**示例类型:** 

- 返回值为枚举类型
- 传入参数为普通类型

**step 1:**

type_def.h

```
enum class StandardOutputRunState : int
{
    NONE,      // 标准输出状态未定义
    STOP_LOW,  // 低电平指示工程停止
    STOP_HIGH, // 高电平指示机器人停止
    RUNNING,   // 指示工程正在运行
};
```

**step 2:**

```
StandardOutputRunState getDigitalOutputRunstate(uint32_t index) override
{
     return client_.CallMethodNamed<StandardOutputRunState>(jsonrpccxx::get_uuid(), "IoControl.getDigitalOutputRunstate",
            { { "index", index } });
}
```

**step 3:**

```
DEF(getDigitalOutputRunstate, { "index" });
```

**step 4:**

```
EXPECT_CALL(*ioc_, getDigitalOutputRunstate(_)).WillRepeatedly(testing::Return(StandardOutputRunState::NONE));
```

**step 5:**

```
TEST_F(MockInterfaceTest, getDigitalOutputRunstate)
{
    EXPECT_EQ(cli->getIoControl()->getDigitalOutputRunstate(1),StandardOutputRunState::NONE);
}
```

<br/>

> #### Demo5:
>

**接口函数:** 自定义结构体

**示例类型:** 

**step 1:**

type_def.h

```
struct RtdeOutputMap
{
    double frequency;
    std::vector<std::string> recipe;
};

inline void to_json(nlohmann::json &j, const RtdeOutputMap &p)
{
    j.at("frequency") = p.frequency;
    j.at("recipe") = p.recipe;
}

inline void from_json(const nlohmann::json &j, RtdeOutputMap &p)
{
    j.at("frequency").get_to(p.frequency);
    j.at("recipe").get_to(p.recipe);
}
```

**step 2:**

常规操作

**step 3:**

常规操作

**step 4:**

常规操作

**step 5:**

常规操作