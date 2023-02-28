function get_vis(host::IPAddr = ip"127.0.0.1", default_port=8700)
    vis = Visualizer(MeshCat.CoreVisualizer(host, default_port), ["meshcat"])
    open(vis)
    return vis
end

function configure_car!(mvis, state, joints, config)
    wb = joints[1]
    lsl = joints[6]
    rsl = joints[7]
    
    axis_roll = [1, 0, 0]
    axis_pitch = [0, 1, 0]
    axis_yaw = [0, 0, 1]
    
    q1 = [cos(config.roll_angle/2); sin(config.roll_angle/2)*axis_roll]
    q2 = [cos(config.pitch_angle/2); sin(config.pitch_angle/2)*axis_pitch]
    q3 = [cos(config.yaw_angle/2); sin(config.yaw_angle/2)*axis_yaw]
    s1 = q1[1]
    s2 = q2[1]
    s3 = q3[1]
    v1 = q1[2:4]
    v2 = q2[2:4]
    v3 = q3[2:4]
    s = s1*s2 - v1'*v2
    v = s1*v2+s2*v1+v1×v2
    q = [s; v]
    s12 = q[1]
    v12 = q[2:4]
    s = s12*s3 - v12'*v3
    v = s12*v3+s3*v12+v12×v3
    q = [s; v]

    set_configuration!(state, wb, [q; config.position])
    set_configuration!(state, lsl, config.steering_angle)
    set_configuration!(state, rsl, config.steering_angle)
    set_configuration!(mvis, configuration(state))
end

function steering_control!(torques::AbstractVector, t, state::MechanismState; reference_angle=0.0, k₁=-6500.0, k₂=-2500.0)
    js = joints(state.mechanism)
    linkage_left = js[6]
    linkage_right = js[7]

    torques[velocity_range(state, linkage_left)] .= k₁ * (configuration(state, linkage_left) .- reference_angle) + k₂ * velocity(state, linkage_left)
    torques[velocity_range(state, linkage_right)] .= k₁ * (configuration(state, linkage_right) .- reference_angle) + k₂ * velocity(state, linkage_right)
end

function suspension_control!(torques::AbstractVector, t, state::MechanismState; k₁=-6500.0, k₂=-2500.0, k₃ = -17500.0, k₄=-5000.0)
    js = joints(state.mechanism)
    front_axle_mount = js[2]
    rear_axle_mount = js[3]
    front_axle_roll = js[4]
    rear_axle_roll = js[5]

    torques[velocity_range(state, front_axle_mount)] .= k₁ * configuration(state, front_axle_mount) + k₂ * velocity(state, front_axle_mount)
    torques[velocity_range(state, rear_axle_mount)] .= k₁ * configuration(state, rear_axle_mount) + k₂ * velocity(state, rear_axle_mount)
    torques[velocity_range(state, front_axle_roll)] .= k₃ * configuration(state, front_axle_roll) + k₄ * velocity(state, front_axle_roll)
    torques[velocity_range(state, rear_axle_roll)] .= k₃ * configuration(state, rear_axle_roll) + k₄ * velocity(state, rear_axle_roll)
end

function view_car(vis; max_realtime_rate=1.0)
    delete!(vis)
    #delete!(vis["/Grid"])
    #delete!(vis["/Axes"])
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy = parse_urdf(urdf_path, floating=true)
    origin = RigidBodyDynamics.Point3D(bodies(chevy)[1].frame_definitions[1].from, [0.0,0,0])
    one_up = RigidBodyDynamics.Point3D(bodies(chevy)[1].frame_definitions[1].from, [0.0,0,1.0])
    dir_up = RigidBodyDynamics.FreeVector3D(one_up)

    hs = RigidBodyDynamics.HalfSpace3D(origin, dir_up)
    RigidBodyDynamics.add_environment_primitive!(chevy, hs)

    # define frictionless contact point on wheel origins (will need to change
    # this for sloped surfaces   
    zero_friction_model = RigidBodyDynamics.ViscoelasticCoulombModel{Float64}(0,1.0,1.0)
    normal_contact_model = RigidBodyDynamics.hunt_crossley_hertz() # investigate parameters if penetrating
    soft_contact_model = RigidBodyDynamics.SoftContactModel(normal_contact_model, zero_friction_model)

    axle = bodies(chevy)[6] 
    for frame_id in 2:3
        frame = axle.frame_definitions[frame_id].from
        pt = RigidBodyDynamics.Point3D(frame, SVector(0.0,-1.25,0))
        contact_point = RigidBodyDynamics.ContactPoint(pt, soft_contact_model)
        RigidBodyDynamics.add_contact_point!(axle, contact_point)
    end
    for (link_id, offset) in zip((7,8), (0.2,-0.2))
        link = bodies(chevy)[link_id]
        frame = link.frame_definitions[2].from
        pt = RigidBodyDynamics.Point3D(frame, SVector(0.0,-1.25,offset))
        contact_point = RigidBodyDynamics.ContactPoint(pt, soft_contact_model)
        RigidBodyDynamics.add_contact_point!(link, contact_point)
    end


    state = MechanismState(chevy)

    wheel_angular_vel = 3.0
    drive_force = 2000.0
    steering_angle = 0.5

    for jid in 8:11
        wheel_joint = joints(chevy)[jid]
        set_velocity!(state, wheel_joint, -wheel_angular_vel)
    end
    
    chevy_visuals = URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))])
    
    mvis = MechanismVisualizer(chevy, chevy_visuals, vis)

    all_joints = joints(chevy)
    config = CarConfig(SVector(0.0,0,2.5), 0.0, 0.0, 0.0, 0.0)
    configure_car!(mvis, state, all_joints, config)


    control! = (torques, t, state) -> begin
        torques .= 0.0
        steering_control!(torques, t, state; reference_angle=steering_angle)
        suspension_control!(torques, t, state)
    end


    @infiltrate
    wrenches! = (bodyid_to_wrench, t, state) -> begin
        RigidBodyDynamics.update_transforms!(state)
        for i = 7:8
            bodyid = BodyID(i)
            wheel = bodies(chevy)[i]
            frame = wheel.frame_definitions[1].from
            body_to_root = transform_to_root(state, bodyid, false)
            wrench = Wrench{Float64}(frame, [0.0,0,0], [drive_force,0,0])
            bodyid_to_wrench[BodyID(i)] = transform(wrench, body_to_root)
        end
        nothing
    end
    
    vehicle_simulate(state, mvis, 10.0, control!, wrenches!; max_realtime_rate)
    #ts, qs, vs = RigidBodyDynamics.simulate(state, 5.0, control!)
    #MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.0)
    #display(qs[end])
end

struct CarConfig
    position::SVector{3, Float64}
    roll_angle::Float64
    pitch_angle::Float64
    yaw_angle::Float64
    steering_angle::Float64
end

function spawn_car(vis, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_id)
    chevy = deepcopy(chevy_base)
    chevy.graph.vertices[2].name="chevy_$vehicle_id"
    state = MechanismState(chevy)
    mvis = MechanismVisualizer(chevy, chevy_visuals, vis)

    println("car_spawned")

    function drive_control!(torques::AbstractVector, t, x::MechanismState)
        0    
    end


    while isopen(sock)
        car_config = Serialization.deserialize(sock)
        configure_car!(mvis, state, chevy_joints, car_config)
        println("updated config")
    end
end
        

function car_sim_server(vis=nothing, host::IPAddr = IPv4(0), port=4444)
    if isnothing(vis)
        vis = get_vis()
    end
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy_base = parse_urdf(urdf_path, floating=true)
    chevy_visuals = URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))])
    chevy_joints = joints(chevy_base)

    vehicle_count = 0
    errormonitor(@async begin
        server = listen(host, port)
        while true
            sock = accept(server)
            println("Client accepted!", sock)
            errormonitor(
                         @async spawn_car(vis, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_count+=1))
        end
    end)
end


