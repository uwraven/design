classdef actuator	
	properties
		min
		max
		speed
		x
		dx
	end
	
	methods
		
		% Initializer
		function self = actuator(range, speed)
			if length(range) ~= 2
				if length(range) < 2
					error("actuator range has only 1 endpoint (2 required)")
				elseif length(range) > 2
					warning("actuator range has more than 2 entries, using min/max values")
				end
			end
			self.min = min(range);
			self.max = max(range);
			self.speed = speed;
		end
	end
end

