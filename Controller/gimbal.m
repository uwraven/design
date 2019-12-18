classdef gimbal < handle
	%GIMBAL Summary of this class goes here
	%   Detailed explanation goes here
	
	properties
		x_act
		y_act
		alpha
		beta
	end
	
	methods
		function self = gimbal(speed, range)
			%GIMBAL Construct an instance of this class
			%   Detailed explanation goes here
			self.x_act = actuator(range, speed);
			self.y_act = actuator(range, speed);
		end
		
	end
end

