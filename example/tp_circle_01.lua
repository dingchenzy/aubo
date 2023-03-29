local aubo = require('aubo')
local sched = sched or aubo.sched
local math = aubo.math or math

local sleep = sched.sleep
local thread = sched.thread
local sync = sched.sync
local run = sched.run
local kill = sched.kill
local halt = sched.halt
function p_tp_circle_01()
    setCollisionStopType(0)
    setCollisionLevel(6)
    modbusDeleteAllSignals()
    setDigitalInputActionDefault()
    setDigitalOutputRunstateDefault()
    setPayload(0, {0,0,0}, {0,0,0}, {0,0,0,0,0,0})
    setTcpOffset({0,0,0,0,0,0})
    Base = {0,0,0,0,0,0}
    Tool = {0,0,0,0,0,0}
    u8defu70b9_0_p={-0.2036419657546438,-0.4367864941141434,-0.02036412895245715,2.629067714573266,0.09753833537625974,2.188922231744193}    
    u8defu70b9_0_q={-1.982658854247186,-0.2962664427061854,1.448712409878828,-0.2355460884584837,1.902041266752724,-2.695795198019681}    
    u8defu70b9_1_p={-0.1802473721489697,-0.4870567228974322,-0.02037457925717001,2.629087663915897,0.09751552430944238,1.537545731479834}    
    u8defu70b9_1_q={-1.978136660705404,-0.1716504648158083,1.6849809482893,0.1708163976800662,2.079969970797575,-2.000417778719469}    
    u8defu70b9_2_p={-0.2373816459279428,-0.5342864665585602,-0.0203413974209557,2.629048104352262,0.09756167800555042,0.0618113270577892}    
    u8defu70b9_2_q={-1.775204373671321,0.0005970397702420957,2.054279132736946,1.001877842116814,1.617930487077249,-0.2792873228061139}    
    u8defu70b9_3_p={-0.2791818549950322,-0.4794235181719664,-0.02031820873643009,2.623825696568802,0.09973917942015369,-1.528374839370064}    
    u8defu70b9_3_q={-1.590348613834722,-0.1417801058492345,1.736742765498084,0.2433920004307756,1.04794827426828,1.499603570614338}    
    u8defu70b9_4_p={-0.1842796956629039,-0.4477119733710719,-0.02028559309108813,2.623828601406457,0.0996643643474973,2.736187501744269}    
    u8defu70b9_4_q={-1.838475280604926,-0.3924970066995653,1.295285435896256,-0.3883781446705054,1.72499655746492,3.213016495536716}    
    
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
        setPlanContext(sched.current_thread_id(), 3, "路点_0")
        setTcpOffset({0,0,0,0,0,0})
        moveLine(poseTrans(Base, u8defu70b9_0_p), 1.2, 0.25, 0, 0)
        
        setPlanContext(sched.current_thread_id(), 4, "圆")
        setCirclePathMode(0)
        setPlanContext(sched.current_thread_id(), 5, "路点_1")
        setTcpOffset({0,0,0,0,0,0})
        
        setPlanContext(sched.current_thread_id(), 6, "路点_2")
        setTcpOffset({0,0,0,0,0,0})
        moveCircle(poseTrans(Base, u8defu70b9_1_p), poseTrans(Base, u8defu70b9_2_p), 1.2, 0.25, 0.0005, 0)
        
        setPlanContext(sched.current_thread_id(), 7, "圆")
        setCirclePathMode(0)
        setPlanContext(sched.current_thread_id(), 8, "路点_3")
        setTcpOffset({0,0,0,0,0,0})
        
        setPlanContext(sched.current_thread_id(), 9, "路点_4")
        setTcpOffset({0,0,0,0,0,0})
        moveCircle(poseTrans(Base, u8defu70b9_3_p), poseTrans(Base, u8defu70b9_4_p), 1.2, 0.25, 0.0005, 0)
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
  p_tp_circle_01()
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


