classdef rcsnozzle < handle
	properties
		thrust
		firing
		should_fire
		min_duration
		cur_duration
		pos
		dir
	end
	
	methods
		function self = rcsnozzle(thrust, min_duration, pos, dir)
			self.thrust = thrust;
			self.min_duration = min_duration;
			self.pos = pos;
			self.dir = dir;
			self.firing = false;
		end
		
		function pulse(self)
			
		end
		
		function tick(self, dt)
			self.cur_duration = self.cur_duration + dt;
			if (self.cur_duration >= self.min_duration)
				% minimum fire duration has been exceeded
				if (~self.should_fire)
					% turn off command has been issued
				end
			end
		end
	end
end

