classdef Gimbal < handle
	
	properties
		x_act
		y_act
		gm
		x_target
		max_angle
	end

	properties (Access = private)
		d
		du
	end
	 
	methods
		function self = Gimbal(speed, range, geometry)

			self.x_act = actuator(range, speed);
			self.y_act = actuator(range, speed);
			
			self.gm = struct(...
				"lc", geometry(1),...
				"l1", geometry(2),...
				"l2", geometry(3));
			
			self.x_act.place([self.gm.l1, 0, 0]);
			self.y_act.place([0, self.gm.l1, 0]);

			self.max_angle = pi / 8;
			
			% distance between actuator centers, geometry is fixed
			self.d = sqrt( (self.x_act.origin(1) - self.y_act.origin(1))^2 + (self.x_act.origin(2) - self.y_act.origin(2))^2 + (self.x_act.origin(3) - self.y_act.origin(3))^2 );

			% unit vector between actuators
			self.du = (self.x_act.origin - self.y_act.origin) / self.d;
			
		end
		
		function setAtOrigin(self)
			
			% initial actuator length
			len = sqrt(self.gm.lc ^ 2 + (self.gm.l1 - self.gm.l2) ^ 2);
			
			self.x_act.setX(len);
			self.y_act.setX(len);

			self.x_target = [0 0];

		end
		
		function getThrustVector(self)
			c1 = self.x_act.origin;
			c2 = self.y_act.origin;

			% r1 = self.x_act.x;
			% r2 = self.y_act.x;

			% % Trilaterate the true direction of the thrust chamber from intersecting spheres

			% % distance from s1 origin to intersecting plane along du
			% xc = (self.d^2 + r1^2 - r2^2) / (2 * self.d);
			
			% % vector from temp origin to intersecting plane
			% vi = xc * self.du;

			




		end
		
	end
end

