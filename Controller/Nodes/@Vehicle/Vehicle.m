classdef Vehicle < handle

	properties (Access = public)

		% Vehicle plant properties
		m = 10;
		cf = 1;
		J = diag(ones(3));
		JInv = diag(ones(3));

		% Guidance and control
		trajectoryPlanner
		controller
		allocator

		% Hardware
		engine
		rcs
	end


	properties (GetAccess = public, SetAccess = private)
		% x is the state vector containing global position, velocity, error quaternion, and body frame angular rates
		% uRequested is the input vector containing global forces and body frame moments computed by the controller
		% uAllocated is the computed allocation using a linear or nonlinear strategy
		% uGlobalRealized is the realized global frame forces and body frame moments resulting from converted actuator commands, this is used to perform integration
		% r is the reference state: position, velocity, stability frame quaternion, body frame angular rates
		x
		uAllocated
		uRequested
		uGlobalRealized
	end
	

	methods (Access = public)

		function self = Vehicle()

			% Guidance and Control
			self.trajectoryPlanner = TrajectoryPlanner();
			self.controller = Controller();
			self.allocator = Allocator();

			% Hardware
			self.gimbal = Gimbal();
			self.engine = Engine();
			self.rcs = ReactionControl();

		end


		function update(self, dt)

			% Update trajectory
			self.trajectoryPlanner.update(dt);

			% Compute requested inputs from controller
			self.uRequested = self.controller.inputs(self.x, self.trajectoryPlanner.filteredCoordinate);

			% Allocate inputs
			self.uAllocated = self.allocator.linearAllocation(self.x, self.uRequested);

			% Realize actuator inputs
			self.uGlobalRealized = self.recoverAllocatedInputs();
			
			self.integratePlant(dt);
		end


		function setState(self, x)
			qs = Quaternion.fromVec(x(7:9));
			X0 = reshape(x, length(x), 1);
			self.x = [X0(1:6); qs; X0(10:12)];
			self.rFiltered = X0;
		end

		
		function setLQRGains(self, K)
			self.control_lqr.setGains(K);
		end

	end

	methods (Access = private)
		function integratePlant(self, dt)
			self.x = RK4(@(x, dt) self.plantDynamics(x, dt), self.x, dt);
			self.x(7:10) = self.x(7:10) / norm(self.x(7:10));
			self.m = self.m - self.cf * self.uAllocated(1) * dt;
		end

		function dX = plantDynamics(self, x, dt)
			dX = [
				reshape(x(4:6), 3, 1)
				reshape(self.uGlobalRealized(1:3) / self.m + [-9.81 0 0]', 3, 1)
				reshape(1 / 2 * skew4(x(11:13)) * x(7:10), 4, 1)
				reshape(self.JInv * self.uGlobalRealized(4:6), 3, 1)
			];
		end

		function U_g = recoverAllocatedInputs(self)
			% Converts an allocated set of actuator commands to forces in the global frame
			U_g = [
				self.uAllocated(1)
				self.uAllocated(2) + self.uAllocated(6)
				self.uAllocated(3) + self.uAllocated(5)
				2 * self.uAllocated(4) * self.allocator.L_R2
				self.uAllocated(5) * self.allocator.L_R1 + self.uAllocated(3) * self.allocator.L_E
				self.uAllocated(6) * self.allocator.L_R1 + self.uAllocated(2) * self.allocator.L_E
			];
		end
	end

	methods
		function set.J(self, J)
			self.J = J;
			self.JInv = J^-1;
		end
	end
	
end

