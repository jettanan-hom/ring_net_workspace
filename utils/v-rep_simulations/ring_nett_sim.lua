function sumsqrt(value)
    -- print("CallBack!")
    return math.sqrt(value[1]^2+value[2]^2+value[3]^2)
end

function setGraph_cb(msg)
    -- print("CallBack!")
    data = msg.data
    sim.setGraphUserData(graphHandle, "Data1", data[1])
    sim.setGraphUserData(graphHandle, "Data2", data[2])
end

function setJointPositions_cb(msg)
    -- print("joint Position CallBack!")
    data = msg.data
    i = data[1] +1
    -- print(i)
    sim.setJointTargetPosition(jointHandle_set[i][1], data[2])
    sim.setJointTargetPosition(jointHandle_set[i][2], data[3])
    sim.setJointTargetPosition(jointHandle_set[i][3], data[4])
    sim.setJointTargetPosition(jointHandle_set[i][4], data[5])
    sim.setJointTargetPosition(jointHandle_set[i][5], data[6])
    sim.setJointTargetPosition(jointHandle_set[i][6], data[7])
    sim.setJointTargetPosition(jointHandle_set[i][7], data[8])
    sim.setJointTargetPosition(jointHandle_set[i][8], data[9]) 
end

function sliderChange(ui,id,newVal)
    if id == 10+0 then
        simUI.setLabelText(ui,1000+0,'Value: '..newVal)
        testParameters[1] = newVal*degToRad
    elseif id == 10+1 then
        simUI.setLabelText(ui,1000+1,'Value: '..newVal)
        testParameters[2] = newVal*degToRad
    elseif id == 10+2 then
        simUI.setLabelText(ui,1000+2,'Value: '..newVal)
        testParameters[3] = newVal*degToRad
    elseif id == 20+0 then
        simUI.setLabelText(ui,2000+0,'Value: '..newVal)
        testParameters[4] = newVal*degToRad
    elseif id == 20+1 then
        simUI.setLabelText(ui,2000+1,'Value: '..newVal)
        testParameters[5] = newVal*degToRad
    elseif id == 20+2 then
        simUI.setLabelText(ui,2000+2,'Value: '..newVal)
        testParameters[6] = newVal*degToRad
    end
end


function sysCall_init()
    -- print("TEST")
    -- do some initialization here
    --spbody = sim.getObjectHandle("Support_Body_joint")

    dt = sim.getSimulationTimeStep()
    segment_num = 3
    stepCounter = 0
    degToRad = math.pi/180
    radToDeg = 180/math.pi

    jointHandle_set={n=segment_num}
    gyroCommunicationTube={n=segment_num}
    accelCommunicationTube={n=segment_num}
    incline={n=segment_num}
    accel={n=segment_num}
    irRangeSensorHandle={n=segment_num}
    footContactSensorHandle={n=segment_num}
    local count = 0
    while count < segment_num do
        local i = count+1
        BC_L_seg = sim.getObjectHandle("BC_joint_legL"..tostring(i))
        CF_L_seg = sim.getObjectHandle("CF_joint_legL"..tostring(i))
        FT_L_seg = sim.getObjectHandle("FT_joint_legL"..tostring(i))
        BC_R_seg = sim.getObjectHandle("BC_joint_legR"..tostring(i))
        CF_R_seg = sim.getObjectHandle("CF_joint_legR"..tostring(i))
        FT_R_seg = sim.getObjectHandle("FT_joint_legR"..tostring(i))
        BBV_seg = sim.getObjectHandle("BBV_joint_seg"..tostring(count))
        BBH_seg = sim.getObjectHandle("BBH_joint_seg"..tostring(count))

        jointHandle_set[i] = {BC_L_seg, CF_L_seg, FT_L_seg,
                                    BC_R_seg, CF_R_seg, FT_R_seg, BBV_seg, BBH_seg}

        gyroCommunicationTube[i]=sim.tubeOpen(0,'gyroData_seg'..tostring(count)..sim.getNameSuffix(nil),1) -- put this in the initialization phase
        incline[i] = {0,0,0}
        
        accelCommunicationTube[i]=sim.tubeOpen(0,'accelerometerData_seg'..tostring(count)..sim.getNameSuffix(nil),1) -- put this in the initialization phase
        accel[i] = {0,0,0}

        irRangeSensorHandle[i] = {sim.getObjectHandle("Proximity_sensorL"..tostring(i)), 
                                        sim.getObjectHandle("Proximity_sensorR"..tostring(i))}

        footContactSensorHandle[i] = {sim.getObjectHandle("Force_sensorL"..tostring(i)), 
                                            sim.getObjectHandle("Force_sensorR"..tostring(i))}

        count = count +1
    end 

    print(accelCommunicationTube[1])


    graphHandle=sim.getObjectHandle("Graph")					--Graph Handle
    testgraphHandle=sim.getObjectHandle("testGraph")					--Graph Handle


    -- TestParameters
    testParameters = {0,0,0,0,0,0}

    -- create each segment ROS Topic
    local jointPositionCommandTopicName={n=segment_num} -- we add a random component so that we can have several instances of this robot running
    local jointPositionTopicName={n=segment_num} -- we add a random component so that we can have several instances of this robot running
    local jointTorqueTopicName={n=segment_num}
    local imuTopicName={n=segment_num}
    -- local electroMagneticCommandTopicName={n=segment_num}
    -- local electroMagneticTopicName={n=segment_num}
    local irSensorTopicName={n=segment_num}
    local footContactTopicName={n=segment_num}
    -- create each segment ROS Node
    jointPositionCommandSub={n=segment_num}
    jointPositionPub={n=segment_num}
    jointTorquePub={n=segment_num}
    imuPub={n=segment_num}
    -- electroMagneticCommandSub={n=segment_num}
    -- electroMagneticPub={n=segment_num}
    irSensorPub={n=segment_num}
    footContactPub={n=segment_num}
    
    if simROS then
        print("<font color='#0F0'>ROS interface was found.</font>@html")

        --local sysTime=sim.getSystemTimeInMs(-1)
        local count = 0
        while count < segment_num do
            local i = count+1
            local tempstr = "seg"..tostring(count)
            jointPositionCommandTopicName[i]= tempstr..'/jointPositionCommand' -- we add a random component so that we can have several instances of this robot running
            jointPositionTopicName[i]= tempstr..'/jointPosition' -- we add a random component so that we can have several instances of this robot running
            jointTorqueTopicName[i]= tempstr..'/jointTorque'
            imuTopicName[i]= tempstr..'/imu'
            -- electroMagneticCommandTopicName[count+1]= tempstr..'/electroMagneticCommand'
            -- electroMagneticTopicName[count+1]= tempstr..'/electroMagnetic'
            irSensorTopicName[i]= tempstr..'/irSensor'
            footContactTopicName[i]= tempstr..'/footContact'
            count = count +1
        end 
        --local FCFeedbackTopicName='/FC_signal' -- we add a random component so that we can have several instances of this robot running
        -- local footHallSensorTopicName='/footHallSensor'
        local testParameterTopicName='/testParameter'
        local terminateControllerName='/terminateController'
        local stepCounterTopicName='/stepCounter'
        local plotterTopicName='/plotter'

        -- Prepare the sensor publisher and the motor speed subscribers:
        local count = 0
        while count < segment_num do
            local i = count+1
            jointPositionCommandSub[i]=simROS.subscribe('/'..jointPositionCommandTopicName[i],
                                                                    'std_msgs/Float32MultiArray','setJointPositions_cb')
            jointPositionPub[i]=simROS.advertise('/'..jointPositionTopicName[i],'std_msgs/Float32MultiArray')
            jointTorquePub[i]=simROS.advertise('/'..jointTorqueTopicName[i],'std_msgs/Float32MultiArray')
            imuPub[i]=simROS.advertise('/'..imuTopicName[i],'std_msgs/Float32MultiArray')
            -- electroMagneticCommandSub[count+1]=simROS.subscribe('/'..electroMagneticCommandTopicName[count+1],'std_msgs/Float32MultiArray','setElectroMagneticState_cb')
            -- electroMagneticPub[count+1]=simROS.advertise('/'..electroMagneticTopicName[count+1],'std_msgs/Float32MultiArray')
            irSensorPub[i]=simROS.advertise('/'..irSensorTopicName[i],'std_msgs/Float32MultiArray')
            footContactPub[i]=simROS.advertise('/'..footContactTopicName[i],'std_msgs/Float32MultiArray')
            count = count+1
        end 
        -- footHallSensorPub=simROS.advertise('/'..footHallSensorTopicName,'std_msgs/Float32MultiArray')
        testParameterPub=simROS.advertise('/'..testParameterTopicName,'std_msgs/Float32MultiArray')
        --FCFeedbackPub=simROS.advertise('/'..FCFeedbackTopicName,'std_msgs/Float64MultiArray')
        -- Utils publisher
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        stepCounterPub=simROS.advertise('/'..stepCounterTopicName,'std_msgs/Int32')
        
        -- ROS subscriber
        -- Util Subscriber
        plotterSub=simROS.subscribe('/'..plotterTopicName,'std_msgs/Float32MultiArray','setGraph_cb')

        -- Control parameter
        MI = 0.001
        
        BiasH1 = 0.01
        BiasH2 = 0.01

        activityH1 = 0.1
        activityH2 = 0.1

        outputH1 = 0.01
        outputH2 = 0.01
        
        WeightH1_H1 = 1.4
        WeightH2_H2 = 1.4
        WeightH1_H2 = 0.22 - MI
        WeightH2_H1 = -0.22 - MI

        -- User Interface setup
        xml = [[ <ui closeable="false" resizable="true" style="plastique" title="Freelander UI" layout="grid">
        <label text="BC_L" wordwrap="true" />
        <label text="Value: 0" id="1000" wordwrap="true" />
        <hslider id="10" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />

        <label text="CF_L" wordwrap="true" />
        <label text="Value: 0" id="1001" wordwrap="true" />
        <hslider id="11" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />

        <label text="FT_L" wordwrap="true" />
        <label text="Value: 0" id="1002" wordwrap="true" />
        <hslider id="12" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
        
        <br />
        <label text="BC_R" wordwrap="true" />
        <label text="Value: 0" id="2000" wordwrap="true" />
        <hslider id="20" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />

        <label text="CF_R" wordwrap="true" />
        <label text="Value: 0" id="2001" wordwrap="true" />
        <hslider id="21" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />

        <label text="FT_R" wordwrap="true" />
        <label text="Value: 0" id="2002" wordwrap="true" />
        <hslider id="22" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
        </ui> ]]

        ui=simUI.create(xml)
        simUI.setPosition(ui, 1200,100)
        
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end



end

function sysCall_actuation()
    -- put your actuation code here

    -- Test CPG Code ----------------------------------------------
    --activityH1 = WeightH1_H1*outputH1+WeightH1_H2*outputH2+BiasH1
    --activityH2 = WeightH2_H2*outputH2+WeightH2_H1*outputH1+BiasH2
    --outputH1 = math.tanh(activityH1)
    --outputH2 = math.tanh(activityH2)
    --sim.setJointTargetPosition(Hip_L_joint, -outputH1/2)
    --sim.setJointTargetPosition(Hip_R_joint, outputH2/2)
    ----------------------------------------------------------------
    
end

function sysCall_sensing()
    -- put your sensing code here

    -- Send an updated sensor and simulation time message, and send the transform of the robot:
    if simROS then
        
        local count = 0
        while count < segment_num do    
            -- read joints position
            local i = count+1
            joint_pos_seg = {   sim.getJointPosition(jointHandle_set[i][1]), 
                                sim.getJointPosition(jointHandle_set[i][2]),
                                sim.getJointPosition(jointHandle_set[i][3]),
                                sim.getJointPosition(jointHandle_set[i][4]),
                                sim.getJointPosition(jointHandle_set[i][5]),
                                sim.getJointPosition(jointHandle_set[i][6]),
                                sim.getJointPosition(jointHandle_set[i][7]),
                                sim.getJointPosition(jointHandle_set[i][8])}
            -- read joints Torque 
            joint_torque_seg = {sim.getJointForce(jointHandle_set[i][1]), 
                                sim.getJointForce(jointHandle_set[i][2]),
                                sim.getJointForce(jointHandle_set[i][3]),
                                sim.getJointForce(jointHandle_set[i][4]),
                                sim.getJointForce(jointHandle_set[i][5]),
                                sim.getJointForce(jointHandle_set[i][6]),
                                sim.getJointForce(jointHandle_set[i][7]),
                                sim.getJointForce(jointHandle_set[i][8])}

            -- To read data from this gyro sensor in another script, use following code:
            datagyro=sim.tubeRead(gyroCommunicationTube[i])
            if (datagyro) then
                angularVariations=sim.unpackFloatTable(datagyro)
            end
            -- print(angularVariations)
            incline[i][1] = incline[i][1] + angularVariations[1]*dt/0.5236*30
            incline[i][2] = incline[i][2] + angularVariations[2]*dt/0.5236*30
            incline[i][3] = incline[i][3] + angularVariations[3]*dt/0.5236*30
            
            -- To read data from this accelerometer in another script, use following code:
            dataaccel=sim.tubeRead(accelCommunicationTube[i])
            if (dataaccel) then
                acceleration=sim.unpackFloatTable(dataaccel)
            end
            
            imu_data={incline[i][1], incline[i][2], incline[i][3], acceleration[1], acceleration[2], acceleration[3]}
       
        
            -- Ir Proximity Sensors
            resultIrL =sim.readProximitySensor(irRangeSensorHandle[i][1])
            resultIrR =sim.readProximitySensor(irRangeSensorHandle[i][2])
            irSensor_data = {resultIrL, resultIrR}
        
            -- footContact Sensors
            rl,forcel,torquel=simReadForceSensor(footContactSensorHandle[i][1])
            rr,forcer,torquer=simReadForceSensor(footContactSensorHandle[i][2])
            fl_ = forcel[3]
            fr_ = forcer[3]
            footContact_data={fl_, fr_}
            
            table.insert(joint_pos_seg, 1, count)
            table.insert(joint_torque_seg, 1, count)
            table.insert(imu_data, 1, count)
            table.insert(irSensor_data, 1, count)
            table.insert(footContact_data, 1, count)

            simROS.publish(jointPositionPub[i], {data=joint_pos_seg})
            simROS.publish(jointTorquePub[i], {data=joint_torque_seg})
            simROS.publish(imuPub[i],{data=imu_data})
            simROS.publish(irSensorPub[i], {data=irSensor_data})
            simROS.publish(footContactPub[i], {data=footContact_data})
            simROS.publish(testParameterPub, {data=testParameters})
            
            count = count +1
        end 

        --Step counter syncing with controller
        stepCounter = stepCounter + 1
        result3=simROS.publish(stepCounterPub, {data=stepCounter})

        
    end   

end

function sysCall_cleanup()
    -- do some clean-up here
    
    -- Terminate Controller
    simROS.publish(terminateControllerPub,{data=true})    
    -- Wait for the signal to reach the node
    waitTimer=0
    while( waitTimer < 1000 ) do
        waitTimer = waitTimer+1
        simROS.publish(terminateControllerPub,{data=true})
    end

    -- simROS.shutdownPublisher(FCFeedbackPub)
    local count = 0
    while count < segment_num do
        local i = count+1
        simROS.shutdownSubscriber(jointPositionCommandSub[i])
        simROS.shutdownPublisher(jointPositionPub[i])
        simROS.shutdownPublisher(jointTorquePub[i])
        simROS.shutdownPublisher(imuPub[i])
        simROS.shutdownPublisher(irSensorPub[i])
        simROS.shutdownPublisher(footContactPub[i])
        -- simROS.shutdownSubscriber(electroMagneticCommandSub[count+1])
        count = count +1
    end 

    -- simROS.shutdownPublisher(footHallSensorPub)
    simROS.shutdownPublisher(stepCounterPub)
    simROS.shutdownPublisher(terminateControllerPub)
    simROS.shutdownSubscriber(plotterSub)

end

-- See the user manual or the available code snippets for additional callback functions and details
