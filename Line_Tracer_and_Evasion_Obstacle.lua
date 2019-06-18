-- IT Applied System Engineering Department 1694021 Song Min-joo
-- 1694021 송민주 코드

function sysCall_init()
         
    bubbleRobBase=sim.getObjectAssociatedWithScript(sim.handle_self)
    leftMotor=sim.getObjectHandle("bubbleRob_leftMotor")
    rightMotor=sim.getObjectHandle("bubbleRob_rightMotor")
    noseSensor=sim.getObjectHandle("bubbleRob_sensingNose")
    rightSensor=sim.getObjectHandle("bubbleRob_sensingRight")
    leftSensor=sim.getObjectHandle("bubbleRob_sensingLeft")
    -- sidesensor(right or left) which detects the obstacle for evasion & returning
	-- rightSensor와 leftSensor는 장애물 복귀를 위해 새로 추가된 센서입니다.
    minMaxSpeed={50*math.pi/180,300*math.pi/180}
    backUntilTime=-1 -- Tells whether bubbleRob is in forward or backward mode
    floorSensorHandles={-1,-1,-1}
    floorSensorHandles[1]=sim.getObjectHandle("leftSensor")
    floorSensorHandles[2]=sim.getObjectHandle("middleSensor")
    floorSensorHandles[3]=sim.getObjectHandle("rightSensor")
    -- Create the custom UI:
    xml = '<ui title="'..sim.getObjectName(bubbleRobBase)..' speed" closeable="false" resizeable="false" activate="false">'..[[
                <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
            <label text="" style="* {margin-left: 300px;}"/>
        </ui>
        ]]
    ui=simUI.create(xml)
    speed=(minMaxSpeed[1]+minMaxSpeed[2])*0.5
    simUI.setSliderValue(ui,1,100*(speed-minMaxSpeed[1])/(minMaxSpeed[2]-minMaxSpeed[1]))
    
end

function speedChange_callback(ui,id,newVal)
    speed=minMaxSpeed[1]+(minMaxSpeed[2]-minMaxSpeed[1])*newVal/100
end


function sysCall_actuation() 
    result=sim.readProximitySensor(noseSensor)
    if (result>0) then
        backUntilTime=sim.getSimulationTime()+1
        
    end

    -- added parameter
    result2=sim.readProximitySensor(rightSensor) --sensor on bubble's right
    result3=sim.readProximitySensor(leftSensor) --sensor on bubble's left
    -- added parameter

    -- read the line detection sensors:
    sensorReading={false,false,false}
    for i=1,3,1 do
        result,data=sim.readVisionSensor(floorSensorHandles[i])
        if (result>=0) then
            sensorReading[i]=(data[11]<0.3) -- data[11] is the average of intensity of the image
        end
    end
    
    -- compute left and right velocities to follow the detected line:
    rightV=speed
    leftV=speed

    if sensorReading[1] then
        leftV=0.03*speed
    end
    if sensorReading[3] then
        rightV=0.03*speed
    end
    if sensorReading[1] and sensorReading[3] then
        backUntilTime=sim.getSimulationTime()+1
    end

    if (backUntilTime<sim.getSimulationTime()) then
        -- When in forward mode, we simply move forward at the desired speed
        sim.setJointTargetVelocity(leftMotor,leftV)
        sim.setJointTargetVelocity(rightMotor,rightV)
    else
        -- When in backward mode, we simply backup in a curve at reduced
        sim.setJointTargetVelocity(leftMotor,speed)
        sim.setJointTargetVelocity(rightMotor,speed/4)
    end

    if (result2>0) then
        -- when right sensor operate, Rob should be turned to counterclockwise
		-- 오른쪽 센서에 장애물이 있다는 것을 확인했을 경우, bubble은 장애물을 끼고 반시계 방향으로 회전합니다.
        sim.setJointTargetVelocity(leftMotor,speed)
        sim.setJointTargetVelocity(rightMotor,speed/3.14)
        if sensorReading[1] then
        -- if it find the line on its left side during turning, it should go on the line by turning to the left side
		-- 센싱 도중 왼쪽에서 선을 만나면 시계방향으로 돌아 라인으로 복귀합니다.
            leftV=0.03*speed
            sim.setJointTargetVelocity(leftMotor,speed)
            sim.setJointTargetVelocity(rightMotor,speed)
        end
        if sensorReading[3] then
        -- if it find the line on its right side during turning, it should go on the line by turning to the right side
		-- 센싱 도중 오른쪽에서 선을 만나면 시계방향으로 돌아 라인으로 복귀합니다.
            rightV=0.03*speed
            sim.setJointTargetVelocity(leftMotor,speed)
            sim.setJointTargetVelocity(rightMotor,speed)
        end
    end

    if (result3>0) then
        -- when left sensor operate, Rob should be turned to clockwise
		-- 왼쪽 센서에 장애물이 있다는 것을 확인했을 경우, bubble은 장애물을 끼고 시계 방향으로 회전합니다.
        sim.setJointTargetVelocity(leftMotor,speed/3.14)
        sim.setJointTargetVelocity(rightMotor,speed)
        if sensorReading[1] then
        -- if it find the line on its left side during turning, it should go on the line by turning to the left side
		-- 센싱 도중 왼쪽에서 선을 만나면 시계방향으로 돌아 라인으로 복귀합니다.
            leftV=0.03*speed
            sim.setJointTargetVelocity(leftMotor,speed)
            sim.setJointTargetVelocity(rightMotor,speed)
        end
        if sensorReading[3] then
        -- if it find the line on its right side during turning, it should go on the line by turning to the right side
		-- 센싱 도중 오른쪽에서 선을 만나면 시계방향으로 돌아 라인으로 복귀합니다.
            rightV=0.03*speed
            sim.setJointTargetVelocity(leftMotor,speed)
            sim.setJointTargetVelocity(rightMotor,speed)
        end
    end

end 

function sysCall_cleanup() 
    simUI.destroy(ui)
end 

