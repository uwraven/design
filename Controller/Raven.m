function raven = Raven()

    % Create vehicle object
    vehicle = Vehicle();
    
    %%%%%%%%%% VEHICLE PLANT %%%%%%%%%%

    vehicle.m = 15;
    vehicle.J = diag([2, 2, 2]);
    

    %%%%%%%%%% ACTUATORS %%%%%%%%%%%

    % RCS actuator configuration
    coldGasThrusterMax = 10;        % Maximum RCS thrust
    RCSArm = 0.8;                   % Z Distance from COM to rcs array
    RCSRadialArm = 0.01;            % Radial distance from center of rcs pod to thruster

    vehicle.rcs.setThrusters([
        ColdGasThruster(coldGasThrusterMax, [RCSRadialArm -RCSRadialArm 0], [1 0 0])
        ColdGasThruster(coldGasThrusterMax, [RCSRadialArm RCSRadialArm 0], [1 0 0])
        ColdGasThruster(coldGasThrusterMax, [0 RCSRadialArm 0], [0 1 0])
        ColdGasThruster(coldGasThrusterMax, [-RCSRadialArm RCSRadialArm 0], [-1 0 0])
        ColdGasThruster(coldGasThrusterMax, [-RCSRadialArm -RCSRadialArm 0], [-1 0 0])
        ColdGasThruster(coldGasThrusterMax, [0 -RCSRadialArm 0], [0 -1 0])
    ]);
    vehicle.rcsMountingDistance = RCSArm;

    % Set EngineAssembly properties
    engineThrustMax = 160;			% N
    engineThrustMin = 90;			% N
    specificThrust = 1 / 0.0006;	% N / kg / s
    gimbalRange = 6;				% Maximum gimbal throw (deg)
    engineArm = 0.6;				% Distance to engine from COM (m)
    engineActuatorArm = 0.1;		% Distance from gimbal origin to engine-side linear actuator mount (m)
    vehicleActuatorArm = 0.1;		% Distance from gimbal origin to vehicle-side linear actuator mount (m)
    engineRadialActuatorArm = 0.0;	% Radial distance from engine center to engine-side linear actuator mount (m)

    vehicle.engineMountingDistance = engineArm;
    vehicle.engine.engineArm = engineActuatorArm;
    vehicle.engine.gimbalArm = vehicleActuatorArm;
    vehicle.engine.gimbal.range = deg2rad(gimbalRange);
    vehicle.engine.specificThrust = specificThrust;
    vehicle.engine.thrustRange = [engineThrustMin engineThrustMax];


    %%%%%%%%%%% CONTROLLERS %%%%%%%%%%%

    vehicle.controller = CascadedController();

    vehicle.accelerationLimit = norm([0.4 0.4]);
    vehicle.controller.horizontalAccelerationLimit = 0.4;
    vehicle.controller.verticalAccelerationLimit = 2;

    wnx = 0.8;
    drx = 1.05;
    wnz = 0.4;
    drz = 1.1;

    kpx = 18;
    kdx = 9;
    kix = 2.5;
    Ktx = 0;

    kpz = 12;
    kdz = 7;
    kiz = 3;
    Ktz = 0;

    pq = 18;
    pw = 24;

    vehicle.controller.trajectoryController.x.setGains(kpx, kix, kdx, Ktx);
    vehicle.controller.trajectoryController.y.setGains(kpx, kix, kdx, Ktx);
    vehicle.controller.trajectoryController.z.setGains(kpz, kiz, kdz, Ktz);
    vehicle.controller.trajectoryPrefilter.x.setGains(wnx, drx, kpx, kix, kdx);
    vehicle.controller.trajectoryPrefilter.y.setGains(wnx, drx, kpx, kix, kdx);
    vehicle.controller.trajectoryPrefilter.z.setGains(wnz, drz, kpz, kiz, kdz);
    vehicle.controller.rateController.setGains(pq, pw);

    %%%%%%%%%%% ALLOCATOR %%%%%%%%%%%%%

    vehicle.allocator.setActuatorArms([engineArm, RCSArm, RCSRadialArm]);
    vehicle.allocator.setActuatorLimits([
        -engineThrustMax * sind(gimbalRange) engineThrustMax * sind(gimbalRange)    % Engine assembly X thrust
        -engineThrustMax * sind(gimbalRange) engineThrustMax * sind(gimbalRange)    % Engine assembly Y thrust
        -engineThrustMax -engineThrustMin                     
        -coldGasThrusterMax coldGasThrusterMax        % Coupled RCS Thruster 1
        -coldGasThrusterMax coldGasThrusterMax        % Coupled RCS Thruster 1
        -coldGasThrusterMax coldGasThrusterMax        % Coupled RCS Thruster 1                                     % Engine assembly Z thrust
    ]);
    
    % Return vehicle
    raven = vehicle;

end