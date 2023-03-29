local aubo = require('aubo')
local sched = sched or aubo.sched
local math = aubo.math or math

local sleep = sched.sleep
local thread = sched.thread
local sync = sched.sync
local run = sched.run
local kill = sched.kill
local halt = sched.halt
function p_tp_moveJspeed_01()
    setCollisionStopType(0)
    setCollisionLevel(6)
    modbusDeleteAllSignals()
    setDigitalInputActionDefault()
    setDigitalOutputRunstateDefault()
    setPayload(0, {0,0,0}, {0,0,0}, {0,0,0,0,0,0})
    setTcpOffset({0,0,0,0,0,0})
    Base = {0,0,0,0,0,0}
    Tool = {0,0,0,0,0,0}
    u8defu70b9_0_p={0.4573127072800388,0.3322432258993765,0.3490677386799897,-3.141537312117622,-0.0001600205471655339,1.571109442490199}    
    u8defu70b9_0_q={0.8501898727148683,-0.1933005927412202,1.587973952492456,0.2172396469181168,1.575339702966241,0.847342841821874}    
    u8defu70b9_1_p={0.5488772298440816,-0.1214954963811684,0.2631772344411978,-3.141572907095999,-2.119557359927315e-05,1.570789172584841}    
    u8defu70b9_1_q={0.003846367899134252,-0.2527560438074985,1.754272059005412,0.4428923210171721,1.575067104634025,0.002125337773347561}    
    
    function waitForMotionComplete()
        if sched.current_thread_id() > 0 then return end
        while (getQueueSize() > 0) or (isSteady() == false) do
            sync()
        end
    end
    
    function str_cat(str1, str2)
        return tostring(str1) .. tostring(str2)
    end
    
    local function calculate_point_to_move_towards(feature, direction, position_distance)
        local posDir={direction[1], direction[2], direction[3]}
        if (math.norm(posDir) < 1e-6) then
            return getTargetTcpPose()
        end
        local direction_vector_normalized=math.normalize(posDir)
        local displacement_pose={direction_vector_normalized[1] * position_distance,direction_vector_normalized[2] * position_distance,direction_vector_normalized[3] * position_distance,0,0,0}

        local wanted_displacement_in_base_frame=poseSub(poseTrans(feature, displacement_pose), feature)
        return poseAdd(getTargetTcpPose(), wanted_displacement_in_base_frame)
    end
    setPlanContext(sched.current_thread_id(), 1, "程序")
    while true do
        setPlanContext(sched.current_thread_id(), 2, "关节运动")
        setPlanContext(sched.current_thread_id(), 3, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 0.523599, 0.523599, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 4, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 0.523599, 0.523599, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 5, "关节运动")
        setPlanContext(sched.current_thread_id(), 6, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 1.0472, 0.523599, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 7, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 1.0472, 0.523599, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 8, "关节运动")
        setPlanContext(sched.current_thread_id(), 9, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 1.74533, 0.523599, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 10, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 1.74533, 0.523599, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 11, "关节运动")
        setPlanContext(sched.current_thread_id(), 12, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 0.523599, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 13, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 0.523599, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 14, "关节运动")
        setPlanContext(sched.current_thread_id(), 15, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 1.39626, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 16, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 1.39626, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 17, "关节运动")
        setPlanContext(sched.current_thread_id(), 18, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 1.74533, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 19, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 1.74533, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 20, "关节运动")
        setPlanContext(sched.current_thread_id(), 21, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 0.523599, 1.74533, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 22, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 0.523599, 1.74533, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 23, "关节运动")
        setPlanContext(sched.current_thread_id(), 24, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 1.0472, 1.74533, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 25, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 1.0472, 1.74533, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 26, "关节运动")
        setPlanContext(sched.current_thread_id(), 27, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 1.74533, 1.74533, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 28, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 1.74533, 1.74533, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 29, "关节运动")
        setPlanContext(sched.current_thread_id(), 30, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_1_q, poseTrans(Base, u8defu70b9_1_p)), 1.74533, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 31, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(u8defu70b9_0_q, poseTrans(Base, u8defu70b9_0_p)), 1.74533, 1.0472, 0, 0)
        sched.cancel_point()
    end
end

local plugin = {
  PRIORITY = 1000, -- set the plugin priority, which determines plugin execution order
  VERSION = "0.1",
  VENDOR = "Aubo Robotics",
}

function plugin:start_robot1(env)
  print("start_robot1--- ************"..tostring(env))
  -- 配置脚本环境
  local _ENV = env
  for k,v in pairs(self.api:loadf(1)) do _ENV[k] = v end
  p_tp_moveJspeed_01()
end

function plugin:start(api)
  --
  self.api = api
  print("start---")
  self:start_robot1(_ENV)
end

function plugin:robot_error_handler(name, err)
  --
  print("An error hanppen to robot "..name)
end

-- return our plugin object
return plugin


