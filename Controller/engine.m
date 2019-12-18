classdef engine
	%ENGINE Summary of this class goes here
	%   Detailed explanation goes here
	
	properties
		max_thrust
		min_thrust
		throttle_min
		fuel
		oxidizer
		pc_max
		pc_min
		Fa
		Fd
	end
	
	methods
		function self = engine(max_thrust, throttle_min, fuel, oxidizer)
			%ENGINE Construct an instance of this class
			%   Detailed explanation goes here
			self.max_thrust = max_thrust;
			self.min_thrust = throttle_min * max_thrust;
			self.throttle_min = throttle_min;
			self.fuel = fuel;
			self.oxidizer = oxidizer;
		end
	end
end

