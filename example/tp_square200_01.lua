local aubo = require('aubo')
local sched = sched or aubo.sched
local math = aubo.math or math

local sleep = sched.sleep
local thread = sched.thread
local sync = sched.sync
local run = sched.run
local kill = sched.kill
local halt = sched.halt
function p_tp_square200_01()
    setCollisionStopType(0)
    setCollisionLevel(6)
    modbusDeleteAllSignals()
    setDigitalInputActionDefault()
    setDigitalOutputRunstateDefault()
    setPayload(0, {0,0,0}, {0,0,0}, {0,0,0,0,0,0})
    setTcpOffset({0,0,0,0,0,0})
    Base = {0,0,0,0,0,0}
    Tool = {0,0,0,0,0,0}
    Waypoint_0_p={0.3922915267910022,0.0780331021673282,0.2640412676198393,3.140462271589499,-0.0007330341171704951,1.569168901446164}    
    Waypoint_0_q={0.5060937736509613,0.14183215547023,2.177991896615931,0.4655351144400988,1.569750938804198,0.5059971317221708}    
    Waypoint_1_p={0.5922902717717815,0.07803629308811734,0.2640537169795334,3.140418032776927,-0.0007493104722837506,1.569167465909602}    
    Waypoint_1_q={0.3365620345839097,-0.3417853671379251,1.636936086368075,0.4081084674844176,1.570352495402355,0.3365709166683311}    
    Waypoint_2_p={0.5923065775948216,-0.1342991517511477,0.2640909796398474,3.140291637092528,-0.0008142938380564554,1.569084489791229}    
    Waypoint_2_q={-0.02089945370088485,-0.3646351507549341,1.604607148245068,0.3985569224746709,1.570884359467797,-0.0207903829167604}    
    Waypoint_3_p={0.3923110798599032,-0.1343067129484366,0.264094643465057,3.140276145945104,-0.0008034112662379482,1.56908219612201}    
    Waypoint_3_q={-0.03157574948977812,0.1014018469263998,2.143366651684301,0.471286582402957,1.570539564832269,-0.03159639351534476}    
    
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
        setPlanContext(sched.current_thread_id(), 3, "Waypoint_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(Waypoint_0_q, poseTrans(Base, Waypoint_0_p)), 1.39626, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 4, "Waypoint_1")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(Waypoint_1_q, poseTrans(Base, Waypoint_1_p)), 1.39626, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 5, "Waypoint_2")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(Waypoint_2_q, poseTrans(Base, Waypoint_2_p)), 1.39626, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 6, "Waypoint_3")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(Waypoint_3_q, poseTrans(Base, Waypoint_3_p)), 1.39626, 1.0472, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 7, "Waypoint_0")
        setTcpOffset({0,0,0,0,0,0})
        moveJoint(inverseKinematics(Waypoint_0_q, poseTrans(Base, Waypoint_0_p)), 1.39626, 1.0472, 0, 0)
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
  p_tp_square200_01()
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


