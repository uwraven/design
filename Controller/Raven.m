function raven = Raven()

    % Create vehicle object
    vehicle = Vehicle();
    
    %%%%%%%%%% VEHICLE PLANT %%%%%%%%%%

    vehicle.m = 15;
    vehicle.J = diag(ones(3, 1));
    

    %%%%%%%%%% ACTUATORS %%%%%%%%%%%

    % RCS actuator configuration
    coldGasThrusterMax = 10;        % Maximum RCS thrust
    RCSArm = 0.8;                   % Z Distance from COM to rcs array
    RCSRadialArm = 0.01;            % Radial distance from center of rcs pod to thruster

    vehicle.rcs.setThrusters([
        ColdGasThruster(coldGasThrusterMax, [RCSRadialArm -RCSRadialArm -RCSArm], [1 0 0])
        ColdGasThruster(coldGasThrusterMax, [RCSRadialArm RCSRadialArm -RCSArm], [1 0 0])
        ColdGasThruster(coldGasThrusterMax, [0 RCSRadialArm -RCSArm], [0 1 0])
        ColdGasThruster(coldGasThrusterMax, [-RCSRadialArm RCSRadialArm -RCSArm], [-1 0 0])
        ColdGasThruster(coldGasThrusterMax, [-RCSRadialArm -RCSRadialArm -RCSArm], [-1 0 0])
        ColdGasThruster(coldGasThrusterMax, [0 -RCSRadialArm -RCSArm], [0 -1 0])
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

    xPGain = 1;
    xIGain = 0;
    xDGain = 0;
    zPGain = 1;
    zIGain = 0;
    zDGain = 0;
    attitudeProportionalGain = diag([0, 0, 0]);
    attitudeRateGain = diag([0, 0, 0]);

    vehicle.controller.trajectoryController.x.setGains(xPGain, xIGain, xDGain);
    vehicle.controller.trajectoryController.y.setGains(xPGain, xIGain, xDGain);
    vehicle.controller.trajectoryController.z.setGains(zPGain, zIGain, zDGain);
    vehicle.controller.rateController.setGains(attitudeProportionalGain, attitudeRateGain);


    %%%%%%%%%%% ALLOCATOR %%%%%%%%%%%%%

    vehicle.allocator.setActuatorArms([engineArm, RCSArm, RCSRadialArm]);
    vehicle.allocator.setActuatorLimits([
        0 coldGasThrusterMax        % RCS Thruster 1
        0 coldGasThrusterMax        % RCS Thruster 2
        0 coldGasThrusterMax        % RCS Thruster 3
        0 coldGasThrusterMax        % RCS Thruster 4
        0 coldGasThrusterMax        % RCS Thruster 5
        0 coldGasThrusterMax        % RCS Thruster 6
        -engineThrustMax * sind(gimbalRange) engineThrustMax * sind(gimbalRange)    % Engine assembly X thrust
        -engineThrustMax * sind(gimbalRange) engineThrustMax * sind(gimbalRange)    % Engine assembly Y thrust
        engineThrustMin, engineThrustMax                                            % Engine assembly Z thrust
    ]);
    
    % Return vehicle
    raven = vehicle;

end