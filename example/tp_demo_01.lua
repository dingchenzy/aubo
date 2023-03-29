local aubo = require('aubo')
local sched = sched or aubo.sched
local math = aubo.math or math

local sleep = sched.sleep
local thread = sched.thread
local sync = sched.sync
local run = sched.run
local kill = sched.kill
local halt = sched.halt
function p_tp_demo_01()
    setCollisionStopType(0)
    setCollisionLevel(6)
    modbusDeleteAllSignals()
    setDigitalInputActionDefault()
    setDigitalOutputRunstateDefault()
    setPayload(0, {0,0,0}, {0,0,0}, {0,0,0,0,0,0})
    setTcpOffset({0,0,0,0,0,0})
    var_0 = nil
    tmp = nil
    k = nil
    loop_0 = nil
    loop_1 = nil
    loop_2 = nil
    setPlanContext(sched.current_thread_id(), 1, "初始变量")
    var_0 = 0
    k = 1
    Base = {0,0,0,0,0,0}
    Tool = {0,0,0,0,0,0}
    Waypoint_3_p={0.5488697627203878,-0.1215000000000001,0.2631947853627901,3.141591289447134,0,1.570796326794896}    
    Waypoint_3_q={0,-0.2617973466375418,1.745327891219924,0.4363302752052282,1.570796326794897,0}    
    Waypoint_4_p={0.5370303837099091,-0.04738227865568143,0.2631934352337221,3.141592468658835,1.849309578747446e-07,1.570796326794862}    
    Waypoint_4_q={0.1393191942297096,-0.2083425892173014,1.813743601584347,0.4512900214647051,1.570796117974612,0.1393191942297105}    
    Waypoint_5_p={0.5844556104413052,-0.1158154239906168,0.2631933205602841,3.141592414044891,1.048637420289393e-09,1.570796326794896}    
    Waypoint_5_q={0.009735820694228181,-0.3393578078595582,1.640455075006986,0.4090167955949875,1.57079632341418,0.009735820694228379}    
    Waypoint_1_p={0.5492466681327987,-0.1219287695546951,0.2640194548205645,3.140422115235442,-0.000813744176149247,1.569128606125283}    
    Waypoint_1_q={-5.817310581846062e-05,-0.2618340875464797,1.745272779856518,0.4363082670370029,1.570796326794897,-6.235647663813106e-05}    
    Waypoint_2_p={0.5492112593150704,-0.3915554592421885,0.2639955458119143,3.140510899097157,-0.0007877094810224581,1.569108295820848}    
    Waypoint_2_q={-0.4377250656022127,-0.5222352796443784,1.368562117030635,0.3194742379853706,1.571306182692114,-0.4374930400931274}    
    
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
    setPlanContext(sched.current_thread_id(), 39, "circle")
    function p_circle()
        setPlanContext(sched.current_thread_id(), 40, "直线运动")
        setPlanContext(sched.current_thread_id(), 41, "Waypoint_3")
        setTcpOffset({0,0,0,0,0,0})
        moveLine(poseTrans(Base, Waypoint_3_p), 1.2, 0.25, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 42, "圆")
        setCirclePathMode(0)
        setPlanContext(sched.current_thread_id(), 43, "Waypoint_4")
        setTcpOffset({0,0,0,0,0,0})
        
        setPlanContext(sched.current_thread_id(), 44, "Waypoint_5")
        setTcpOffset({0,0,0,0,0,0})
        moveCircle(poseTrans(Base, Waypoint_4_p), poseTrans(Base, Waypoint_5_p), 1.2, 0.25, 0, 0)
    end
    
    setPlanContext(sched.current_thread_id(), 35, "Thread_0")
    handler_Thread_0=thread(function()
        while true do
            setPlanContext(sched.current_thread_id(), 36, "'Stopped'")
            -- Stopped
            
            setPlanContext(sched.current_thread_id(), 37, "如果 DO00")
            if (getStandardDigitalOutput(0)) then
                setPlanContext(sched.current_thread_id(), 38, "Halt")
                waitForMotionComplete()
                halt()
            end
        end
    end, "Thread_0")
    run(handler_Thread_0)
    setPlanContext(sched.current_thread_id(), 2, "在开始之前")
    setPlanContext(sched.current_thread_id(), 3, "设置 DO00=Off")
    setStandardDigitalOutput(0, false)
    
    setPlanContext(sched.current_thread_id(), 4, "弹窗 消息：Running")
    waitForMotionComplete()
    clearNamedVariable("_popup_continue_or_stop")
    local _popup_t = getControlSystemTime()
    popup(TraceLevel.INFO, "Info", "Running", 0)
    while not variableUpdated("_popup_continue_or_stop", _popup_t) do sync() end
    if not getBool("_popup_continue_or_stop", true) then halt() end
    
    setPlanContext(sched.current_thread_id(), 5, "原点")
    moveJoint({0,-0.2617993877991494,1.74532925199433,0.4363323129985824,1.570796326794897,0}    , 1.39626, 1.0472, 0, 0)
    setPlanContext(sched.current_thread_id(), 6, "程序")
    while true do
        setPlanContext(sched.current_thread_id(), 7, "如果 var_0<=10")
        if (var_0<=10) then
            setPlanContext(sched.current_thread_id(), 8, "关节运动")
            setPlanContext(sched.current_thread_id(), 9, "Waypoint_2")
            setTcpOffset({0,0,0,0,0,0})
            moveJoint(inverseKinematics(Waypoint_2_q, poseTrans(Base, Waypoint_2_p)), 1.39626, 1.0472, 0, 0)
            
            setPlanContext(sched.current_thread_id(), 10, "Waypoint_1")
            setTcpOffset({0,0,0,0,0,0})
            moveJoint(inverseKinematics(Waypoint_1_q, poseTrans(Base, Waypoint_1_p)), 1.39626, 1.0472, 0, 0)
            
            setPlanContext(sched.current_thread_id(), 11, "var_0:=var_0+1")
            var_0 = var_0+1
            
            setPlanContext(sched.current_thread_id(), 12, "否则如果 var_0<=20")
        elseif (var_0<=20) then
            setPlanContext(sched.current_thread_id(), 13, "直线运动")
            setPlanContext(sched.current_thread_id(), 14, "原点")
            moveJoint({0,-0.2617993877991494,1.74532925199433,0.4363323129985824,1.570796326794897,0}            , 1.39626, 1.0472, 0, 0)
            
            setPlanContext(sched.current_thread_id(), 15, "Direction:BaseY+")
            flag_thread_move_15=0
            handler_thread_move_15=thread(function()
                -- enter_critical
                flag_thread_move_15=1
                towardsPos=calculate_point_to_move_towards({0,0,0,0,0,0,}, {0.0,1.0,0.0}, 0.1)
                moveLine(towardsPos, 1.2, 0.25, 0, 0)
                flag_thread_move_15=2
                -- exit_critical
            end, "thread_move_15")
            
            run(handler_thread_move_15)
            while true do
                setPlanContext(sched.current_thread_id(), 16, "Until Distance: 100 mm")
                sleep(1e-10)
                if (flag_thread_move_15 > 1) then
                    kill(handler_thread_move_15)
                    break
                end
                sync()
            end
            
            setPlanContext(sched.current_thread_id(), 17, "var_0:=var_0+1")
            var_0 = var_0+1
            
            setPlanContext(sched.current_thread_id(), 18, "否则")
        else
            setPlanContext(sched.current_thread_id(), 19, "'Rotate k turns'")
            -- Rotate k turns
            
            setPlanContext(sched.current_thread_id(), 20, "tmp:=k")
            tmp = k
            
            setPlanContext(sched.current_thread_id(), 21, "分支 k")
            do
                local cases = {}
                setPlanContext(sched.current_thread_id(), 22, "条件 1")
                cases[1] = function ()
                    setPlanContext(sched.current_thread_id(), 23, "loop_0 1 times")
                    for n=1,1,1 do
                        setPlanContext(sched.current_thread_id(), 24, "Call circle")
                        p_circle()
                        sched.cancel_point()
                    end
                end
                
                setPlanContext(sched.current_thread_id(), 25, "条件 2")
                cases[2] = function ()
                    setPlanContext(sched.current_thread_id(), 26, "loop_1 2 times")
                    for n=1,2,1 do
                        setPlanContext(sched.current_thread_id(), 27, "Call circle")
                        p_circle()
                        sched.cancel_point()
                    end
                end
                
                setPlanContext(sched.current_thread_id(), 28, "条件 3")
                cases[3] = function ()
                    setPlanContext(sched.current_thread_id(), 29, "loop_2 3 times")
                    for n=1,3,1 do
                        setPlanContext(sched.current_thread_id(), 30, "Call circle")
                        p_circle()
                        sched.cancel_point()
                    end
                end
                
                setPlanContext(sched.current_thread_id(), 31, "默认")
                cases.default = function ()
                    setPlanContext(sched.current_thread_id(), 32, "Call circle")
                    p_circle()
                end
                if (cases[k]) then
                    cases[k]()
                elseif (cases.default) then
                    cases.default()
                end
            end
            
            setPlanContext(sched.current_thread_id(), 33, "k:=k+1")
            k = k+1
            
            setPlanContext(sched.current_thread_id(), 34, "等待：3")
            waitForMotionComplete()
            sleep(3)
        end
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
  p_tp_demo_01()
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


