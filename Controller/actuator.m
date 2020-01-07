classdef Actuator < handle
	
	% good pid gains are [0.3 2.0 0.0];
	
	properties
		min
		max
		speed
		origin
		x
		x_target
		gains
	end
	
	properties (Access = private)
		err
		err_integral
		err_previous
	end
	
	methods
		
		% Initializer
		function self = Actuator(range, speed)
			
			% validate range input
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
			
			self.err_integral = 0;
			self.err_previous = 0;
			self.err = 0;
			self.gains = struct(...
				"p", 0.1,...
				"i", 0,...
				"d", 0);
			
			self.x = (self.max - self.min) / 2 + self.min;
			self.x_target = self.x;
			self.origin = [0 0 0];
		end
		
		function place(self, origin)
			self.origin = origin;
		end

		function setX(self, x)
			% Set the actuator extension
			self.x = x;
		end
			
		function setTarget(self, x)
			% Set the actuator target (PI)
			if (x > self.max)
				self.x_target = self.max;
			elseif (x < self.min)
				self.x_target = self.min;
			else
				self.x_target = x;
			end
		end
		
		function actuate(self, dt)
			self.err = self.x_target - self.x;
			self.err_integral = self.err_integral + self.err * dt;
 			% input = self.gains.p * self.err + self.gains.i * self.err_integral + self.gains.d * (self.err - self.err_previous) / dt;
			input = self.gains.p * self.err + self.gains.i * self.err_integral + 0.1;
			
			dx = input / dt;
			if (abs(dx) > self.speed)
				self.x = self.x + self.speed * dt;
			else
				self.x = self.x + input;
			end
			
			if (self.x > self.max)
				self.x = self.max;
			elseif (self.x < self.min)
				self.x = self.min;
			end
			
			self.err_previous = self.err;			
		end
				
		function update(self, dt)
			self.actuate(dt);
		end
	end
end

