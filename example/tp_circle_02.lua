local aubo = require('aubo')
local sched = sched or aubo.sched
local math = aubo.math or math

local sleep = sched.sleep
local thread = sched.thread
local sync = sched.sync
local run = sched.run
local kill = sched.kill
local halt = sched.halt
function p_tp_circle_02()
    setCollisionStopType(0)
    setCollisionLevel(6)
    modbusDeleteAllSignals()
    setDigitalInputActionDefault()
    setDigitalOutputRunstateDefault()
    setPayload(0, {0,0,0}, {0,0,0}, {0,0,0,0,0,0})
    setTcpOffset({0,0,0,0,0,0})
    Base = {0,0,0,0,0,0}
    Tool = {0,0,0,0,0,0}
    Waypoint_0_p={0.3881144528271677,-0.1190436005037465,0.212115162519678,3.138039143704267,-0.005049504846182802,1.569571447419688}    
    Waypoint_0_q={7.336056075074242e-06,-0.2020166441673569,1.813869208786406,0.4450858581308293,1.570869687355647,-0.0001320490093513364}    
    Waypoint_1_p={0.4693518471429214,-0.1190351249740284,0.2121122692838011,3.138094344214076,-0.005062843094875683,1.569713797262075}    
    Waypoint_1_q={-9.536872897596515e-05,-0.4850233474035335,1.424203586274726,0.3383719184347619,1.570550568916381,0.0001393850654264106}    
    Waypoint_2_p={0.4693424758344846,-0.06987959032929038,0.212110532919179,3.138107651800067,-0.005070845481123842,1.569760157193806}    
    Waypoint_2_q={0.1057639204343453,-0.4493187624861472,1.477863168435856,0.3568294355196486,1.570209442308891,0.1058629571913589}    
    
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
        setPlanContext(sched.current_thread_id(), 2, "直线运动")
        setPlanContext(sched.current_thread_id(), 3, "Waypoint_0")
        setTcpOffset({0,0,0,0,0,0})
        moveLine(poseTrans(Base, Waypoint_0_p), 1.2, 0.25, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 4, "圆")
        setCirclePathMode(0)
        setPlanContext(sched.current_thread_id(), 5, "Waypoint_1")
        setTcpOffset({0,0,0,0,0,0})
        
        setPlanContext(sched.current_thread_id(), 6, "Waypoint_2")
        setTcpOffset({0,0,0,0,0,0})
        moveCircle(poseTrans(Base, Waypoint_1_p), poseTrans(Base, Waypoint_2_p), 1.2, 0.25, 0, 0)
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
  p_tp_circle_02()
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


