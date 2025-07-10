module QuadcopterFlightControl
using Zenoh, CDRSerialization
using CDRSerialization: CDRReader
using Rotations, LinearAlgebra
using StaticArrays, GeometryBasics
using PX4: px4

# Drone properties
Base.@kwdef struct QuadrotorModel
    mass::Float64  # [kg] = 1.5 + 0.015 + 0.005*4 for iris
    g::Float64 # [m/s^2] = 9.82
    e3::SVector{3, Int64} = [0; 0; 1]
end

# Geometric control gains
Base.@kwdef struct GeometricControlParams
    kx::Float64
    kv::Float64
end

update_interactive(::Nothing,tgt,real) = GeometricControlParams(kx = 1.0, kv = 1.0)

function circle(interactive_handle=nothing)

    setpoint_kexpr = Zenoh.Keyexpr("0/fmu/in/trajectory_setpoint/px4_msgs::msg::dds_::TrajectorySetpoint_/RIHS01_9a7deb0cb7db5e52b441169a284b90188c4efdf06b302d55cb1db33ca0358c1a")
    mode_kexpr = Zenoh.Keyexpr("0/fmu/in/offboard_control_mode/px4_msgs::msg::dds_::OffboardControlMode_/RIHS01_e87463f252f320d2f197d1feadd643b8645e5ef53a2b158dccfdf11372e00f9f")
    command_kexpr = Zenoh.Keyexpr("0/fmu/in/vehicle_command/px4_msgs::msg::dds_::VehicleCommand_/RIHS01_1f0ab537807e2e20e04826633ea2c52e5431bea8eb0187152ff23bd07f1dd30b")
    
    step=0
    sub_odo = nothing
    sub_failsafes = nothing
    s = nothing
    try
        c = Zenoh.Config(; str = """{connect: { endpoints: ["tcp/172.25.128.1:7448"]}}""")
        s = open(c)
        ready_for_arm = Threads.Atomic{Bool}(false)
        armed = Ref(false)
        start_time = Ref(UInt64(0))
        ref_pos = [0.0, 0.0, 0.0]
        target_pos = [0.0, 0.0, 0.0]
        ref_vel = [NaN, NaN, NaN]
        ref_accel = [NaN, NaN, NaN]
        data = IOBuffer()

        iris_model = QuadrotorModel(mass = 1.5 + 0.015 + 0.005*4, g = 0.0) # g is already compensated by px4
        ctrl_pars = GeometricControlParams(kx = 1.0, kv=1.0)
        function geometric_force_controller(x_des, xd_des, xdd_des, x, v, R, QuadModel::QuadrotorModel, ControlGains::GeometricControlParams)
            kx = ControlGains.kx
            kv = ControlGains.kv
            g = QuadModel.g
            e3 = QuadModel.e3
            mass = QuadModel.mass

            # Errors in state
            ex = x - x_des
            ev = v - xd_des

            # Force Control

            return (-kx * ex - kv * ev - mass * g * e3 + mass * xdd_des)/mass
        end

        pub_mode = Zenoh.Publisher(s, mode_kexpr)
        pub_setpoint = Zenoh.Publisher(s, setpoint_kexpr)
        pub_command = Zenoh.Publisher(s, command_kexpr)
        sub_odo = open((sample) -> begin 
            p = Zenoh.payload(sample)
            str = open(p, Val(:read))
            r = CDRReader(str)
            odometry = read(r, px4.msg.VehicleOdometry)
            if !armed[] 
                ref_pos .= odometry.position
                target_pos .= ref_pos
            else
                elapsed = (odometry.timestamp - start_time[])/1e6
                speed = 1.25
                size = 5.0
                travel_dist = elapsed * speed
                tpos = SVector(size*sin(travel_dist+3π/2)+1 + ref_pos[1], size*cos(travel_dist+3π/2) + ref_pos[2], ref_pos[3])
                rvel = SVector(size*speed*cos(travel_dist+3π/2), -size*speed*sin(travel_dist+3π/2), 0.0)
                target_pos .= tpos
                ref_vel .= rvel
                target_accel = SVector(-size*speed*speed*sin(travel_dist+3π/2), -size*speed*speed*cos(travel_dist+3π/2), 0.0)
                ref_accel .= geometric_force_controller(tpos, rvel, target_accel, odometry.position, odometry.velocity, QuatRotation(1.0, 0.0, 0.0, 0.0), iris_model, ctrl_pars)
                ctrl_pars = update_interactive(interactive_handle, Point3f(target_pos), Point3f(odometry.position)) # GeometricControlParams(kx = sl_kx.value[], kv = sl_kv.value[])
                ref_accel .= SVector(-size*speed*speed*sin(travel_dist+3π/2), -size*speed*speed*cos(travel_dist+3π/2), 0.0)
            end
            cmd_mode=px4.msg.OffboardControlMode(0, false, false, true, false, false, false, false)
            w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR_LE)
            write(w, cmd_mode)
            Zenoh.put(pub_mode, take!(data))
            target = px4.msg.TrajectorySetpoint(;
                timestamp = 0, 
                position = target_pos, velocity = ref_vel, 
                acceleration = ref_accel, jerk = [NaN, NaN, NaN], 
                yaw = NaN, yawspeed = NaN)
            w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR_LE)
            write(w, target)
            Zenoh.put(pub_setpoint, take!(data))
            if ready_for_arm[] && !armed[]
                armed[] = true
                start_time[] = odometry.timestamp
                w = CDRSerialization.CDRWriter(data, CDRSerialization.CDR_LE)
                msg = px4.msg.VehicleCommand(;
                    timestamp=0,
                    command=px4.msg.VehicleCommand_Constants.VEHICLE_CMD_DO_SET_MODE,
                    param1=1, param2=6, param3=0, param4=0,param5=0, param6=0, param7=0,
                    target_system=1, target_component=1, source_system=1, source_component=1, confirmation=0, from_external=true)
                write(w, msg)
                Zenoh.put(pub_command, take!(data))
            end
            close(str)
        end, s, Zenoh.Keyexpr("0/fmu/out/vehicle_odometry/px4_msgs::msg::dds_::VehicleOdometry_/RIHS01_4295b96ad312084aa35c84bdcec65ba2f57eb4f6a293d57d52d95acbe99ff817"))
        sub_failsafes = open((sample) -> begin 
            p = Zenoh.payload(sample)
            str = open(p, Val(:read))
            try 
                r = CDRReader(str)
                failsafes = read(r, px4.msg.FailsafeFlags)
                Threads.atomic_xchg!(ready_for_arm, !failsafes.offboard_control_signal_lost)
                #@show !failsafes.offboard_control_signal_lost
            finally 
                close(str)
            end
        end, s, Zenoh.Keyexpr("0/fmu/out/failsafe_flags/px4_msgs::msg::dds_::FailsafeFlags_/RIHS01_a3b231a285771d620f31efa501aa3b440c9e66e95d2950fbe8617056b33d585f"))
        while (true)
            sleep(0.1) # wait for ctrl-c
        end
    finally
        if !isnothing(sub_odo) close(sub_odo) end
        if !isnothing(sub_failsafes) close(sub_failsafes) end
        if !isnothing(s) close(s) end
    end
end

#= 
=#

end