local aubo = require('aubo')
local sched = sched or aubo.sched
local math = aubo.math or math

local sleep = sched.sleep
local thread = sched.thread
local sync = sched.sync
local run = sched.run
local kill = sched.kill
local halt = sched.halt
function p_tp_moveJ_01()
    setCollisionStopType(0)
    setCollisionLevel(6)
    modbusDeleteAllSignals()
    setDigitalInputActionDefault()
    setDigitalOutputRunstateDefault()
    setPayload(0, {0,0,0}, {0,0,0}, {0,0,0,0,0,0})
    setTcpOffset({0,0,0,0,0,0})
    Base = {0,0,0,0,0,0}
    Tool = {0,0,0,0,0,0}
    Waypoint_0_p={0.4693691998574843,-0.1190258728645832,0.1766748700791839,3.138089569095067,-0.005102633805226828,1.569770968887215}    
    Waypoint_0_q={0.008454984985638382,-0.202050947370162,2.096717263937563,0.7382016195062721,1.579430010250213,0.008678059550837891}    
    Waypoint_1_p={0.4693614113687745,-0.1190308023265567,0.2120983219320084,3.138090919654756,-0.005081556030329008,1.569742807967504}    
    Waypoint_1_q={0.00852087,-0.140041,2.05293,0.632401,1.57953,0.00844993}    
    Waypoint_2_p={0.4693759210556978,0.01613960594339786,0.1766600077319377,3.138094321291525,-0.005122956142505079,1.569797645344643}    
    Waypoint_2_q={0.3001841619098842,-0.1697776919501056,2.138216728487652,0.7487382155283807,1.578128893143266,0.3003897518363348}    
    
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
        setPlanContext(sched.current_thread_id(), 3, "Waypoint_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(Waypoint_1_q, poseTrans(Base, Waypoint_1_p)), 1.39626, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 4, "Waypoint_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(Waypoint_0_q, poseTrans(Base, Waypoint_0_p)), 1.39626, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 5, "Waypoint_2")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(Waypoint_2_q, poseTrans(Base, Waypoint_2_p)), 1.39626, 1.0472, 0, 0)
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
  p_tp_moveJ_01()
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


