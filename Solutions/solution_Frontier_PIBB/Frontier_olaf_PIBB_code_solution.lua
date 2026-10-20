function sysCall_init()
    sim = require('sim')

    -- Joint and leg names
    robot_1 = sim.getObject('/Olaf')
    joint_names = {'joint_1','joint_2','joint_3'}
    leg_names = {'L1','L2','L3','R1','R2','R3'}
    L1_joints = {}
    L2_joints = {}
    L3_joints = {}
    R1_joints = {}
    R2_joints = {}
    R3_joints = {}

    for i = 1, 3, 1 do
        L1_joints[i] = sim.getObject('/' .. joint_names[i] .. '_' .. leg_names[1])
        L2_joints[i] = sim.getObject('/' .. joint_names[i] .. '_' .. leg_names[2])
        L3_joints[i] = sim.getObject('/' .. joint_names[i] .. '_' .. leg_names[3])
        R1_joints[i] = sim.getObject('/' .. joint_names[i] .. '_' .. leg_names[4])
        R2_joints[i] = sim.getObject('/' .. joint_names[i] .. '_' .. leg_names[5])
        R3_joints[i] = sim.getObject('/' .. joint_names[i] .. '_' .. leg_names[6])
    end    
    robot_joints = {L1_joints, L2_joints, L3_joints, R1_joints, R2_joints, R3_joints}

    Left_legs_joint_bias = {-1,  1, 1}
    Right_legs_joint_bias = {-1, 1, 1}


    -- IMU  
    IMU = sim.getObject('/IMU')
    IMU_ref = sim.getObject('/IMU_ref')

    simTime = 0
    lastTime = 0
    round_count = -1


    -- ============================================== --
    -- ============================================== --
    --                  CPG setting                   --
    -- ============================================== --
    -- ============================================== --

    o1 = 0.7856
    o2 = -0.5782
    a1 = 0
    a2 = 0
    MI = 0.1
    w11 = 1.4
    w12 = 0.18 + MI
    w21 = -0.18 - MI
    w22 = 1.4

    -- ============================================== --
    -- ============================================== --
    -- ============================================== --
    --           ____       _   _   _             
    --          / ___|  ___| |_| |_(_)_ __   __ _ 
    --          \___ \ / _ \ __| __| | '_ \ / _` |
    --           ___) |  __/ |_| |_| | | | | (_| |
    --          |____/ \___|\__|\__|_|_| |_|\__, |
    --                                      |___/ 
    -- ============================================== --
    N_rolled_out = 3 -- max is 7
    time_per_round = 7 -- seconds
    -- ============================================== --
    -- ============================================== --
    print("|-----------------------------------------------|")
    print("|---------------PIBB Simulation-----------------|")
    print("|-----------------------------------------------|")
    print("Number of rolled out robots: " .. N_rolled_out)
    print("Time per round: " .. time_per_round .. " seconds")
    print("-------------------------------------------------")
    print(" ")



    copied_robot = {robot_1}
    copied_robot_joints = {robot_joints}
    
    --  ========= > Copy robot
    if (N_rolled_out > 1) then
        for R = 2, N_rolled_out, 1 do
            table.insert(copied_robot, sim.copyPasteObjects({robot_1},1)[1])
            sim.setObjectName(copied_robot[R], 'copied_arm_' .. R)
            sim.setObjectPosition(copied_robot[R], {-2,  4.5-1.5*(R-1), 0.2})
        end -- for
    end -- if

    sim.setObjectPosition(copied_robot[2], -1, {-2,  0, 0.2})

    -- ========= > Copy robot joints and end_effector and target
    if (N_rolled_out > 1) then
        for R = 2, N_rolled_out, 1 do
            -- Joint
            -- print(R .. " || " .. copied_robot[R])
            joints_in_robot = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}}
            for i=1, 3, 1 do
                joints_in_robot[1][i] = sim.getObject('./joint_' .. i .. '_L1', {proxy = copied_robot[R]})
                joints_in_robot[2][i] = sim.getObject('./joint_' .. i .. '_L2', {proxy = copied_robot[R]})
                joints_in_robot[3][i] = sim.getObject('./joint_' .. i .. '_L3', {proxy = copied_robot[R]})
                joints_in_robot[4][i] = sim.getObject('./joint_' .. i .. '_R1', {proxy = copied_robot[R]})
                joints_in_robot[5][i] = sim.getObject('./joint_' .. i .. '_R2', {proxy = copied_robot[R]})
                joints_in_robot[6][i] = sim.getObject('./joint_' .. i .. '_R3', {proxy = copied_robot[R]})
            end
            table.insert(copied_robot_joints, joints_in_robot)

        end -- for
    end -- if

    robot_joints_position = {}
    for R = 1, N_rolled_out, 1 do
        table.insert(robot_joints_position,{{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}})
    end

    first_iteration = 0
    resample_state = true
    calculate_state = false

    -- ============================================== --
    --                      Graph                     --
    -- ============================================== --
    graphHandle = sim.getObject('/graph')


    -- Add data streams (e.g., for X and Y coordinates)
    streamX = sim.addGraphStream(graphHandle, 'X-Axis Data', 'm', 0, {1, 0, 0})
    streamY = sim.addGraphStream(graphHandle, 'Y-Axis Data', 'm', 0, {0, 1, 0})
    
    
    -- Create the 2D curve connecting the streams
    -- curveId = sim.addGraphCurve(graphHandle, "2D Trajectory", 2, {streamX, streamY}, {0,0}, 'm', 4, {0, 0, 1}, 2)
    -- 2. Define the XY curve
    sim.addGraphCurve(graphHandle, 'Param X vs Y', 2, {streamX, streamY}, {0, 0}, 'm', 0, {1, 0, 0})

    
    -- ============================ --
    --             PIBB             --
    -- ============================ --
    PIBB_initial(N_rolled_out)
end -- sysCall_init()



    -- ################################################# --
    -- ################################################# --
    -- ################################################# --
    --         .______    __  .______   .______          --
    --         |   _  \  |  | |   _  \  |   _  \         --
    --         |  |_)  | |  | |  |_)  | |  |_)  |        --
    --         |   ___/  |  | |   _  <  |   _  <         --
    --         |  |      |  | |  |_)  | |  |_)  |        --
    --         | _|      |__| |______/  |______/         --

    -- ################################################# --
    -- ################################################# --
    -- ################################################# --


function PIBB_initial(N)


    -- ▄▀▀▄      █ █▄  █ █ ▀▀█▀▀ █ ▄▀▀▄ █   
    -- █  █      █ █ ▀▄█ █   █   █ █▀▀█ █    
    --  ▀▀       ▀ ▀   ▀ ▀   ▀   ▀ ▀  ▀ ▀▀▀▀ 
    -- ============================================= --
    -- ============= EDIT code Here ================ --
    -- ============================================= --

    current_policy = {0, 0}
    w1_current = 0
    w2_current = 0

    rolled_out_policies = {}
    noises = {}
    mean = 0
    std_dev = 0.2
    decay_noise_variation = 0.995
    learning_rate = 0.1


    -- ============================================= --
end -- initial function



function PIBB_algorithm()
    calculate_state = PIBB_reset(time_per_round)
    -- print(calculate_state)
    -- ============================= --
    --   Sample in parameter space   --
    -- ============================= --
    if resample_state == true then
        round_count = round_count + 1
        if round_count > 0 then
            print("Round: " .. round_count .. " | Time: " .. sim.getSimulationTime())
            print("Current policy (w1, w2): " .. w1_current .. ", " .. w2_current)
        else
            print("Initial rollout of policies")
        end
        resample_state = false


        -- ============================================================================= --
        -- ============================================================================= --

        --    ▀█        ▄▀▀▀ ▄▀▀▄ █▄ ▄█ █▀▀▄ █    █▀▀▀      █▄  █ ▄▀▀▀▄ █ ▄▀▀▀ █▀▀▀
        --     █         ▀▀▄ █▀▀█ █ ▀ █ █▀▀  █    █▀▀       █ ▀▄█ █   █ █  ▀▀▄ █▀▀ 
        --     ▀        ▀▀▀  ▀  ▀ ▀   ▀ ▀    ▀▀▀▀ ▀▀▀▀      ▀   ▀  ▀▀▀  ▀ ▀▀▀  ▀▀▀▀
        -- ============================================================================= --
        -- ============================================================================= --
        noises = {}
        -- ===== >>>> Sample noise for each rolled out policy <<<< ==== --
        for R=1, N_rolled_out, 1 do
            i_noise_1 = ((2*math.random()) - 1) * std_dev + mean                   -- random_gauss(mean, std_dev)
            i_noise_2 = ((2*math.random()) - 1) * std_dev + mean                   -- random_gauss(mean, std_dev)
            a_rolled_out_noise = {i_noise_1, i_noise_2}
            table.insert(noises, a_rolled_out_noise)
        end
        
        -- the noise is look like
        -- noises = {  {#####, #####}, --for robot 1
        --             {#####, #####}, --for robot 2
        --             {#####, #####}, --for robot 3
        --                 .
        --                 .
        --                 .
        --             {#####, #####}} --for robot N


        -- ============================================================================= --
        -- ============================================================================= --

        -- ▀▀▀▄      ▄▀▀▄ █▀▀▄ █▀▀▄      █▄  █ ▄▀▀▀▄ █ ▄▀▀▀ █▀▀▀      ▀▀█▀▀ ▄▀▀▀▄ 
        -- ▄▀▀       █▀▀█ █  █ █  █      █ ▀▄█ █   █ █  ▀▀▄ █▀▀         █   █   █ 
        -- ▀▀▀▀      ▀  ▀ ▀▀▀  ▀▀▀       ▀   ▀  ▀▀▀  ▀ ▀▀▀  ▀▀▀▀        ▀    ▀▀▀  

        -- ▄▀▀▄ █  █ █▀▀▄ █▀▀▄ █▀▀▀ █▄  █ ▀▀█▀▀      █▀▀▄ ▄▀▀▀▄ █    █ ▄▀▀▄ ▀▄ ▄▀
        -- █  ▄ █  █ █▀▀▄ █▀▀▄ █▀▀  █ ▀▄█   █        █▀▀  █   █ █    █ █  ▄   █  
        --  ▀▀   ▀▀  ▀  ▀ ▀  ▀ ▀▀▀▀ ▀   ▀   ▀        ▀     ▀▀▀  ▀▀▀▀ ▀  ▀▀    ▀  
        -- ============================================================================= --
        -- ================================= EDIT code Here ============================ --
        -- ============================================================================= --
        -- ==== >>>> Add noise to current policy <<<< ==== --
        rolled_out_policies = {}
        -- Add your code here to add noise to
        -- the current policy and create rolled_out_policies
        -- The rolled_out_policies MUST look like this !!!!!!!
        -- rolled_out_policies = { {CP1+#####, CP2+#####}, --for robot 1
        --                         {CP1+#####, CP2+#####}, --for robot 2
        --                         {CP1+#####, CP2+#####}, --for robot 3
        --                             .
        --                             .
        --                             .
        --                         {CP1+#####, CP2+#####}} --for robot N
        -- =============================================================================== --
        -- for each robot, add noise to the current policy and store it in rolled_out_policies
        for R=1, N_rolled_out, 1 do 
            w1_each_robot = w1_current + noises[R][1]
            w2_each_robot = w2_current + noises[R][2]
            each_robot_policy = {w1_each_robot, w2_each_robot}
            -- Add robot policy to rolled_out_policies
            table.insert(rolled_out_policies, each_robot_policy)
        end







        -- ============================================= --
    end -- Sample in parameter space


    -- ============================================================================= --
    -- ============================================================================= --

    -- ▀▀▀▄      █▀▀▀ ▀▄ ▄▀ █▀▀▀ ▄▀▀▄ █  █ ▀▀█▀▀ █▀▀▀ 
    --  ▀▀▄      █▀▀   ▄▀▄  █▀▀  █  ▄ █  █   █   █▀▀  
    -- ▀▀▀       ▀▀▀▀ ▀   ▀ ▀▀▀▀  ▀▀   ▀▀    ▀   ▀▀▀▀ 
    --
    -- █▀▀▄ ▄▀▀▀▄ █    █    █▀▀▀ █▀▀▄      ▄▀▀▀▄ █  █ ▀▀█▀▀      █▀▀▄ ▄▀▀▀▄ █    █ ▄▀▀▄ █ █▀▀▀ ▄▀▀▀
    -- █▀▀▄ █   █ █    █    █▀▀  █  █      █   █ █  █   █        █▀▀  █   █ █    █ █  ▄ █ █▀▀   ▀▀▄
    -- ▀  ▀  ▀▀▀  ▀▀▀▀ ▀▀▀▀ ▀▀▀▀ ▀▀▀        ▀▀▀   ▀▀    ▀        ▀     ▀▀▀  ▀▀▀▀ ▀  ▀▀  ▀ ▀▀▀▀ ▀▀▀     
    -- ============================================================================= --
    -- ============================================================================= --
    -- ============================= --
    --        Execute policy         --
    -- ============================= --
    if not resample_state then
        -- ============= Execute policy ================ --
        run_robot_second(rolled_out_policies)
        -- ============================================= --
    end -- Execute policy on Robots

    -- ============================================================================= --
    -- ============================= --
    --  Calculate and Update policy  --
    -- ============================= --
    if calculate_state == true and sim.getSimulationTime() > 1 then
        resample_state = true
        -- ============================================================================= --
        -- ============================================================================= --

        -- █  █      ▄▀▀▀  █▀▀▀ ▀▀█▀▀      █▀▀▄ ▄▀▀▀▄ █▀▀▄ ▄▀▀▀▄ ▀▀█▀▀      █▀▀▄ █ ▄▀▀▀ ▀▀█▀▀ ▄▀▀▄ █▄  █ ▄▀▀▄ █▀▀▀
        -- ▀▀▀█      █  ▀█ █▀▀    █        █▀▀▄ █   █ █▀▀▄ █   █   █        █  █ █  ▀▀▄   █   █▀▀█ █ ▀▄█ █  ▄ █▀▀ 
        --    ▀       ▀▀▀  ▀▀▀▀   ▀        ▀  ▀  ▀▀▀  ▀▀▀   ▀▀▀    ▀        ▀▀▀  ▀ ▀▀▀    ▀   ▀  ▀ ▀   ▀  ▀▀  ▀▀▀▀
        -- ============================================================================= --
        -- ============================================================================= --
        -- ========>>>> Get robot distances <<<<======= --
        dist_array = get_robot_distance(N_rolled_out)
        print("Robot distances: ")
        print(dist_array)

        -- ============================================================================= --
        -- ============================================================================= --

        -- █▀▀▀      ▄▀▀▄ ▄▀▀▄ █    █  █ █    ▄▀▀▄ ▀▀█▀▀ █▀▀▀  
        -- ▀▀▀▄      █  ▄ █▀▀█ █    █  █ █    █▀▀█   █   █▀▀   
        -- ▀▀▀        ▀▀  ▀  ▀ ▀▀▀▀  ▀▀  ▀▀▀▀ ▀  ▀   ▀   ▀▀▀▀  

        -- █▀▀▄ █ ▄▀▀▀ ▀▀█▀▀ ▄▀▀▄ █▄  █ ▄▀▀▄ █▀▀▀      ▄▀▀▄ ▄▀▀▀▄ ▄▀▀▀ ▀▀█▀▀
        -- █  █ █  ▀▀▄   █   █▀▀█ █ ▀▄█ █  ▄ █▀▀       █  ▄ █   █  ▀▀▄   █  
        -- ▀▀▀  ▀ ▀▀▀    ▀   ▀  ▀ ▀   ▀  ▀▀  ▀▀▀▀       ▀▀   ▀▀▀  ▀▀▀    ▀  
        -- ============================================================================= --
        -- ============================================================================= --
        -- ============================================================================= --
        -- ========>>>> Calculate normalized distances <<<<======= --
        -- normalized_array = {0, 0, 0, ... , 0}
        -- min_val_in_array = get_min(arr)
        -- max_val_in_array = get_max(arr)
        lambda = 10
        Rk = {} -- Reward for each rolled out policy
        Sk = {} -- Normalized reward for each rolled out policy
        -- ============================================================================= --
        -- =========================== EDIT code Here ================================== --
        -- ============================================================================= --

        -- Because we can use distance as reward
        Rk = dist_array
        
        -- Normalize the rewards using min-max normalization
        min_Rk = get_min(Rk)
        max_Rk = get_max(Rk)
        for R=1, N_rolled_out, 1 do
            a_normalized_reward = math.exp(lambda * (Rk[R] - min_Rk) / (max_Rk - min_Rk))
            table.insert(Sk, a_normalized_reward) -- add normalized reward to Sk
        end






        -- ================================= Hint ================================== --

        -- ============================================================================= --
        -- ============================================================================= --

        -- ▄▀▀       ▄▀▀▄ ▄▀▀▄ █    █  █ █    ▄▀▀▄ ▀▀█▀▀ █▀▀▀ 
        -- █▀▀▄      █  ▄ █▀▀█ █    █  █ █    █▀▀█   █   █▀▀  
        --  ▀▀        ▀▀  ▀  ▀ ▀▀▀▀  ▀▀  ▀▀▀▀ ▀  ▀   ▀   ▀▀▀▀ 

        -- █▀▀▄ █▀▀▄ ▄▀▀▀▄ █▀▀▄ ▄▀▀▄ █▀▀▄ █ █    █ ▀▀█▀▀ ▀▄ ▄▀
        -- █▀▀  █▀▀▄ █   █ █▀▀▄ █▀▀█ █▀▀▄ █ █    █   █     █  
        -- ▀    ▀  ▀  ▀▀▀  ▀▀▀  ▀  ▀ ▀▀▀  ▀ ▀▀▀▀ ▀   ▀     ▀  
        -- ============================================================================= --
        -- ============================================================================= --
        -- ============================================================================= --
        -- ========>>>> Calculate Probabilities <<<<======= --
        -- Pk = {0, 0, 0, ... , 0}
        Pk = {}
        -- ============================================================================= --
        -- =========================== EDIT code Here ================================== --
        -- ============================================================================= --
        sum_Sk = sum_array(Sk)
        for R=1, N_rolled_out, 1 do
            a_probability = Sk[R] / sum_Sk
            table.insert(Pk, a_probability) -- add probability to Pk
        end



        print("Probabilities: ")
        print(Pk)

        -- ============================================================================= --
        -- ============================================================================= --

        -- ▀▀▀█      ▄▀▀▄ ▄▀▀▀▄ ▄▀▀▀ ▀▀█▀▀         █   █ █▀▀▀ █ ▄▀▀▀  █  █ ▀▀█▀▀ █▀▀▀ █▀▀▄
        --  ▄▀       █  ▄ █   █  ▀▀▄   █    ▀▀▀▀   █ ▄ █ █▀▀  █ █  ▀█ █▀▀█   █   █▀▀  █  █
        -- ▀          ▀▀   ▀▀▀  ▀▀▀    ▀            ▀ ▀  ▀▀▀▀ ▀  ▀▀▀  ▀  ▀   ▀   ▀▀▀▀ ▀▀▀ 

        --   ▄▀▀▄ █   █ █▀▀▀ █▀▀▄ ▄▀▀▄ ▄▀▀▀  █ █▄  █ ▄▀▀▀ 
        --   █▀▀█  █ █  █▀▀  █▀▀▄ █▀▀█ █  ▀█ █ █ ▀▄█ █  ▀█
        --   ▀  ▀   ▀   ▀▀▀▀ ▀  ▀ ▀  ▀  ▀▀▀  ▀ ▀   ▀  ▀▀▀ 
        -- ============================================================================= --
        -- =========================== EDIT code Here ================================== --
        -- ============================================================================= --
        w1_avg = 0
        w2_avg = 0
        -- ========>>>> Cost-weighted averaging <<<<======= --

        for R=1, N_rolled_out, 1 do
            w1_avg = w1_avg + (Pk[R] * noises[R][1])
            w2_avg = w2_avg + (Pk[R] * noises[R][2])
        end

        -- ============================================================================= --
        -- ============================================================================= --
        -- ▄▀▀▄      █  █ █▀▀▄ █▀▀▄ ▄▀▀▄ ▀▀█▀▀ █▀▀▀      █▀▀▄ ▄▀▀▀▄ █    █ ▄▀▀▄ ▀▄ ▄▀ 
        -- ▄▀▀▄      █  █ █▀▀  █  █ █▀▀█   █   █▀▀       █▀▀  █   █ █    █ █  ▄   █   
        --  ▀▀        ▀▀  ▀    ▀▀▀  ▀  ▀   ▀   ▀▀▀▀      ▀     ▀▀▀  ▀▀▀▀ ▀  ▀▀    ▀   

        -- █▀▀▄ ▄▀▀▄ █▀▀▄ ▄▀▀▄ █▄ ▄█ █▀▀▀ ▀▀█▀▀ █▀▀▀ █▀▀▄ ▄▀▀▀
        -- █▀▀  █▀▀█ █▀▀▄ █▀▀█ █ ▀ █ █▀▀    █   █▀▀  █▀▀▄  ▀▀▄
        -- ▀    ▀  ▀ ▀  ▀ ▀  ▀ ▀   ▀ ▀▀▀▀   ▀   ▀▀▀▀ ▀  ▀ ▀▀▀ 
        -- ============================================================================= --
        -- ============================================================================= --
        -- ========>>>> Update policy parameters <<<<======= --

        w1_current = w1_current + (learning_rate * w1_avg)
        w2_current = w2_current + (learning_rate * w2_avg)
        -- ============================================================================= --
        -- ============================================================================= --
        -- ▄▀▀▄      █▀▀▄ █▀▀▀ ▄▀▀▄ ▄▀▀▄ ▀▄ ▄▀      █▀▀▀ ▀▄ ▄▀ █▀▀▄ █    ▄▀▀▀▄ █▀▀▄ ▄▀▀▄ ▀▀█▀▀ █ ▄▀▀▀▄ █▄  █ 
        --  ▀▀█      █  █ █▀▀  █  ▄ █▀▀█   █        █▀▀   ▄▀▄  █▀▀  █    █   █ █▀▀▄ █▀▀█   █   █ █   █ █ ▀▄█ 
        --  ▀▀       ▀▀▀  ▀▀▀▀  ▀▀  ▀  ▀   ▀        ▀▀▀▀ ▀   ▀ ▀    ▀▀▀▀  ▀▀▀  ▀  ▀ ▀  ▀   ▀   ▀  ▀▀▀  ▀   ▀ 

            -- █▄  █ ▄▀▀▀▄ █ ▄▀▀▀ █▀▀▀
            -- █ ▀▄█ █   █ █  ▀▀▄ █▀▀ 
            -- ▀   ▀  ▀▀▀  ▀ ▀▀▀  ▀▀▀▀
        -- ========>>>> Decay exploration noise <<<<======= --
        -- ============================================================================= --
        -- ============================================================================= --
        std_dev = std_dev * decay_noise_variation


    -- ============================= --
    end -- Calculate and Update policy

    -- ******************************************************** --
    -- ******************************************************** --

    -- ============================= --
    --      Reset Robot position     --
    -- ============================= --
    if calculate_state then
        -- ============= Reset robot position ========== --
        reset_robot_position()
        -- ============================================= --
        sim.setGraphStreamValue(graphHandle, streamX, w1_current)
        sim.setGraphStreamValue(graphHandle, streamY, w2_current)
    end -- Execute policy on Robots

end -- PIBB_algorithm()


-- ****************************************************************************** --

-- ▄▀▀▄ █▀▀▄ █▀▀▀ ▄▀▀▄ ▀▀█▀▀ █▀▀▀      █▀▀▀ █  █ █▄  █ ▄▀▀▄ ▀▀█▀▀ █ ▄▀▀▀▄ █▄  █
-- █  ▄ █▀▀▄ █▀▀  █▀▀█   █   █▀▀       █▀▀  █  █ █ ▀▄█ █  ▄   █   █ █   █ █ ▀▄█
--  ▀▀  ▀  ▀ ▀▀▀▀ ▀  ▀   ▀   ▀▀▀▀      ▀     ▀▀  ▀   ▀  ▀▀    ▀   ▀  ▀▀▀  ▀   ▀

-- ******************************************************************************* --

function NameOfYourFunction(param1)
    -- Your code here
end

-- Example of a function
function exampleFunction(param1, data_array)
    -- create a new array to store the results
    an_new_array = {}

    -- for loop from 1 to the length of data_array
    for i = 1, #data_array do
        -- print the current iteration and the value from data_array
        print("Iteration: " .. i)
        -- insert the value from data_array into an_new_array
        -- it is the same as array.append in python
        table.insert(an_new_array, data_array[i])
    end
    -- return the new array
    return an_new_array
end




















-- **************************************************** --
--                       ACTUATION                      --
-- **************************************************** --
function sysCall_actuation()
    -- Sin and cos functions for the legs locomotion
    simtime = sim.getSimulationTime()
    -- o1 = math.sin(simtime*4)*0.2
    -- o2 = math.cos(simtime*4)*0.4

    a1 = o1*w11+ o2*w12
    a2 = o2*w22+ o1*w21

    o1 = tanh(a1)
    o2 = tanh(a2)


    PIBB_algorithm()

end -- sysCall_actuation()





-- **************************************************** --
--                       SENSING                        --  
-- **************************************************** --

function sysCall_sensing()
    
end -- sysCall_sensing()


-- **************************************************** --
--                       CLEANUP                        --  
-- **************************************************** --
function sysCall_cleanup()
    -- do some clean-up here
end -- sysCall_cleanup()


function get_robot_distance(N)
    robot_distances = {}
    for R=1, N_rolled_out, 1 do
        dist = sim.getObjectPosition(copied_robot[R])
        table.insert(robot_distances, dist[1]+2)
    end
    return robot_distances
end

function PIBB_reset(time_per_round)

    currentTime = sim.getSimulationTime()
    if (currentTime - lastTime >= time_per_round) or (first_iteration == 0) then
        first_iteration = 1
        lastTime = currentTime
        return true
    else
        return false
    end
end -- PIBB_reset

function run_robot_second(rolled_out_policies)
    if #rolled_out_policies == 0 then
        local array = {}
        for i=1, 10, 1 do
            table.insert(array, {-1,-1})
        end
        rolled_out_policies = array
    end

    A1 = o1 * 0.5
    A2 = o2 * 0.5

    for R = 1, N_rolled_out, 1 do -- robot
        for L = 1, 6, 2 do -- leg left
            robot_joints_position[R][L][1] = Left_legs_joint_bias[1] * (A1*rolled_out_policies[R][1])
            robot_joints_position[R][L][2] = Left_legs_joint_bias[2] * (A2*rolled_out_policies[R][2]) + math.rad(45)
            robot_joints_position[R][L][3] = math.rad(-135)
        end

        for L = 2, 6, 2 do -- leg right
            robot_joints_position[R][L][1] = Right_legs_joint_bias[1] * (-A1*rolled_out_policies[R][1])
            robot_joints_position[R][L][2] = Right_legs_joint_bias[2] * (-A2*rolled_out_policies[R][2]) + math.rad(45)
            robot_joints_position[R][L][3] = math.rad(-135)
        end
    end

    -- Command to set the joint positions to the values in robot_joints_position
    for R = 1, N_rolled_out, 1 do -- robot
        for L = 1, 6, 1 do -- leg
            for J = 1, 3, 1 do -- joint
                sim.setJointTargetPosition(copied_robot_joints[R][L][J], robot_joints_position[R][L][J])
            end
        end
    end
end -- run_robot_second()

function reset_robot_position()
    -- Reset the robot position to the initial state
    print("---------------End round-----------------")
    print(" ")
    for R=1, N_rolled_out, 1 do 
        sim.setObjectPosition(copied_robot[R], -1, {-2,  4.5-1.5*(R-1), 0.2})
        sim.setObjectOrientation(copied_robot[R], -1, {0, 0, 0})
    end
end -- reset_robot_position()

function example_softMax(vals, temperature)
    local sum = 0
    for i=1, #vals do
        sum = sum + math.exp(vals[i] / temperature)
    end

    local softmax_vals = {}
    for i=1, #vals do
        softmax_vals[i] = math.exp(vals[i] / temperature) / sum
    end
    return softmax_vals
end

function random_gauss(mean, std_dev)
    local u1 = math.random()
    local u2 = math.random()
    
    -- Box-Muller transform formula
    local rand_std_normal = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    
    -- Scale by standard deviation and shift by the mean
    return mean + (rand_std_normal * std_dev)
end

function get_min(arr)
    return math.min(table.unpack(arr))
end

function get_max(arr)
    return math.max(table.unpack(arr))
end

function sum_array(arr)
    local sum = 0
    for i=1, #arr do
        sum = sum + arr[i]
    end
    return sum
end

function tanh(x)
    return (math.exp(x) - math.exp(-x))/(math.exp(x) + math.exp(-x))
end -- tanh function