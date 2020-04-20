classdef GimbalActuator < handle
% GimbalActuator converts desired gimbal angles to actuator inputs needed.
% Outer ring of the gimbal corresponds to yaw angle rotation while the 
% inner ring corresponds to the pitch angle rotation.

properties (Access = public)  
end

properties (Access = private)
end

properties (SetAccess = private, GetAccess = public)
    arms  % arms(1)->D, arms(2)->d,arms(3)->L1,arms(4)->L2...
    % arms(5)->h
    theta % the angle between the actuator arm and x-y plane
    ang_y % desired gimbal angle 1 - yaw
    ang_p % desired gimbal angle 2 - pitch
    act_o
    act_i  
    
end

methods (Access = public)
    function actgimbal = GimbalActuator(arms,theta,ang_y,ang_p)
        actgimbal.arms = arms;
        actgimbal.theta = theta;
        actgimbal.ang_y = ang_y;
        actgimbal.ang_p = ang_p;
        
        fun_o = @(dL) arms(1).*cos(ang_y) - arms(2).*sin(ang_y) +....
          (arms(3)+dL).*cos(asin(((arms(1)*sin(ang_y))+ arms(2).*...
          cos(ang_y)-arms(5))./(arms(3)+dL)))-arms(1)-arms(3).*cos(ang_y);
      
        fun_i = @(dL) arms(1).*cos(ang_y) - arms(2).*sin(ang_y) +....
          (arms(4)+dL).*cos(asin(((arms(1)*sin(ang_y))+ arms(2).*...
          cos(ang_y)-arms(5))./(arms(4)+dL)))-arms(1)-arms(4).*cos(ang_y);
       
        act_o = fsolve(@(dL) fun_o(dL), 0);
        act_i = fsolve(@(dL) fun_i(dL), 0);
      
        actgimbal.act_o = act_o;
        actgimbal.act_i = act_i;
        
    end
end
end