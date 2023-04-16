struct MyLocalizationType
    time::Float64
    lat::Float64
    long::Float64
    yaw::Float64
    vel::Float64
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end

function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    @info "localize start"
    while true
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end
        
        # process measurements
        @info time()

        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end

function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        # write h to turn simple state vehicle to BOUNDING box
        # return a percept type that gives time as well as list of means of seen vehicles
        latest_localization_state = fetch(localization_state_channel)
        
        # process bounding boxes / run ekf / do what you think is good

        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        map, 
        target_road_segment_id, 
        socket)
    # do some setup
    while true
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        # figure out what to do ... setup motion planning problem etc
        steering_angle = 0.0
        target_vel = 0.0
        cmd = VehicleCommand(steering_angle, target_vel, true)
        serialize(socket, cmd)
    end
end

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    @async while true
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        local measurement_msg
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
            else
                break
            end
        end
        target_map_segment = measurement_msg.target_segment
        ego_vehicle_id = measurement_msg.vehicle_id
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
                @info meas
            end
        end
    end

    @async localize(gps_channel, imu_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end



function mushroom_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    running = true
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)
    msg = deserialize(socket) # Visualization info
    @info msg

    map_segments = training_map()
    max_id = 0
    for seg in map_segments
        max_id = max(max_id, seg[2].id)
    end
    seg_lengths = build_seg_lengths(map_segments)
    mid_lengths = build_mid_lengths(map_segments, max_id)
    adj_matrix = build_adjacency_matrix(map_segments, max_id)

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)
    target_channel = Channel{MeasurementMessage}(1)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 82 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    localization_state = MyLocalizationType(-100,0.0,0.0,0.0,0.0)

    @async while isopen(socket) && running
        sleep(0.0001)
        local measurement_msg
        measurement_msg = deserialize(socket)
        #target_map_segment = measurement_msg.target_segment
        #@info target_map_segment
        ego_vehicle_id = measurement_msg.vehicle_id
        measurements = measurement_msg.measurements
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
  #      @info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
    end

    @async while isopen(socket) && running
        sleep(0.001)
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end
        fresh_gt_meas = []
        while isready(gt_channel)
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end
        
        # process measurements


        if !isempty(fresh_gt_meas)
            gt = fresh_gt_meas[length(fresh_gt_meas)]
            yaw = get_yaw(gt.orientation)
            vel = norm(gt.velocity)
            localization_state = MyLocalizationType(gt.time,gt.position[1],gt.position[2],yaw,vel)

            if isready(localization_state_channel)
                take!(localization_state_channel)
            end
            put!(localization_state_channel, localization_state)
        end
    end

    # first localization
    while !isready(localization_state_channel)
        sleep(0.01)
    end
    localized_coords = take!(localization_state_channel)
    @info localized_coords
    seg = get_current_segment(localized_coords, map_segments)[1]
    @info seg

    # decision-making
    path = get_path(seg, target_map_segment, map_segments, seg_lengths, max_id)
    @info path
    cur_index = length(path)
    @info target_map_segment
    @async while isopen(socket) && running
        sleep(0.001)
        if isready(localization_state_channel)
            localized_coords = take!(localization_state_channel)
        end
        
        arr = get_current_segment(localized_coords, map_segments)
        if path[cur_index-1] in arr
            cur_index-=1
        end

        yaw = localized_coords.yaw
        cmd = VehicleCommand(0, 0.3, true)
        i = 0
        while cur_index-i > 0 && mid_lengths[path[cur_index-i],1] < 5
            i+=1
        end
        i = cur_index-i == 0 ? i-1 : i
        target_vel = norm([localized_coords.lat, localized_coords.long] - [map_segments[path[cur_index-i]].lane_boundaries[1].pt_a[1],map_segments[path[cur_index-i]].lane_boundaries[1].pt_a[2]])
        target_vel /= 6
        target_vel = target_vel > 10 ? 10 : target_vel
        speed = target_vel + (target_vel - localized_coords.vel)
        speed = speed > 10 ? 10 : speed
        speed = speed < 5 ? 5 : speed
        if abs(map_segments[path[cur_index]].lane_boundaries[1].curvature) < 0.001
            dist_from_mid = 0
            if mid_lengths[path[cur_index],1] == 1
                dist_from_mid = localized_coords.long - mid_lengths[path[cur_index],2]
                target_yaw = -atan(dist_from_mid/20)
                steer = target_yaw - yaw
            elseif mid_lengths[path[cur_index],1] == 2
                #up
                dist_from_mid = localized_coords.lat - mid_lengths[path[cur_index],2]
                target_yaw = yaw_converter(pi/2 + atan(dist_from_mid/20))
                steer = get_steer!(target_yaw, yaw)
            elseif mid_lengths[path[cur_index],1] == 3
                #left
                dist_from_mid = localized_coords.long - mid_lengths[path[cur_index],2]
                target_yaw = atan(dist_from_mid/20)
                steer = target_yaw - yaw
            else
                #down
                dist_from_mid = localized_coords.lat - mid_lengths[path[cur_index],2]
                target_yaw = yaw_converter(-pi/2 - atan(dist_from_mid/20))
                steer = get_steer!(target_yaw, yaw)
            end
            if abs((yaw+6*pi)%(pi/2) - pi/2) > 0.05 || dist_from_mid > 2
                speed = speed > 5 ? 5 : speed
            end
        elseif map_segments[path[cur_index]].lane_boundaries[1].curvature > 0
            #@info "curve-left"
            speed = 3

            circle_spot = [localized_coords.lat, localized_coords.long] - [mid_lengths[path[cur_index], 3],mid_lengths[path[cur_index], 4]]
            circle_spot_atan = atan(circle_spot[2], circle_spot[1])
            tangent_yaw = yaw_converter(circle_spot_atan + pi/2)
            error = norm(circle_spot) - mid_lengths[path[cur_index], 2]
            target_yaw = yaw_converter(tangent_yaw + atan(error/20))
            steer = get_steer!(target_yaw, yaw)
            steer += 0.2

        elseif map_segments[path[cur_index]].lane_boundaries[1].curvature < 0
            #@info "curve-right"
            speed = 3

            circle_spot = [localized_coords.lat, localized_coords.long] - [mid_lengths[path[cur_index], 3], mid_lengths[path[cur_index], 4]]
            circle_spot_atan = atan(circle_spot[2], circle_spot[1])
            tangent_yaw = yaw_converter(circle_spot_atan - pi/2)
            error = norm(circle_spot) - mid_lengths[path[cur_index], 2]
            target_yaw = yaw_converter(tangent_yaw - atan(error/20))
            steer = get_steer!(target_yaw, yaw)
            steer -= 0.2
        end

        if steer > 1
            steer = 1
        elseif steer < -1
            steer = -1
        end
        cmd = VehicleCommand(steer, speed, true)
        serialize(socket, cmd)
    end

    
    #@async localize(gps_channel, imu_channel, localization_state_channel)
    #@async perception(cam_channel, localization_state_channel, perception_state_channel)
    #@async decision_making(localization_state_channel, perception_state_channel, map, socket)

    # keyboard
    target_velocity = 0.0
    steering_angle = 0.0
    controlled = true
    @info "Press 'q' at any time to terminate vehicle."
    while controlled && isopen(socket) && running
        key = get_c()
        if key == 'q'
            # terminate vehicle
            controlled = false
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Keyboard Client."
            running = false
        elseif key == 'i'
            # increase target velocity
            target_velocity += v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'k'
            # decrease forward force
            target_velocity -= v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'j'
            # increase steering angle
            steering_angle += s_step
            @info "Target steering angle: $steering_angle"
        elseif key == 'l'
            # decrease steering angle
            steering_angle -= s_step
            @info "Target steering angle: $steering_angle"
        end
        #cmd = VehicleCommand(steering_angle, target_velocity, controlled)
        #serialize(socket, cmd)
    end
end


