classdef Vehicle < handle

	properties (Access = public)

		% Vehicle plant properties
		m = 10;
		J = diag(ones(3));
		JInv = diag(ones(3));
		engineMountingDistance;
		rcsMountingDistance;

		% Guidance and control
		trajectoryPlanner
		estimator
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

		% Vehicle, allocator, and controller states
		x
		uAllocated
		uRequestedGlobal
		uRealizedGlobal
	end
	

	methods (Access = public)

		function self = Vehicle()

			% Guidance and Control
			self.trajectoryPlanner = TrajectoryPlanner();
			self.controller = CascadedController();
			self.allocator = Allocator();

			% Hardware
			self.engine = EngineAssembly();
			self.rcs = ReactionControl();

		end


		function update(self, dt)

			% Update control
			self.trajectoryPlanner.update(dt);
			self.controller.update(dt);
			self.allocator.update(dt);

			% Compute requested inputs from controller
			uRequestedGlobal = self.controller.inputs(self.x, self.trajectoryPlanner.filteredCoordinate);
			self.uRequestedGlobal = uRequestedGlobal;

			% Convert requested inputs from global to local frame
			uRequestedLocal = self.getLocalRequest();

			% Allocate inputs
			uAllocated = self.allocator.linearAllocation(uRequestedLocal);
			self.uAllocated = uAllocated;

			self.engine.setAllocation(uAllocated(1:3));
			self.rcs.setAllocation(uAllocated(4:end));

			% Update actuator plants
			self.engine.update(dt);
			self.rcs.update(dt);
			
			% Realize actuator inputs
			self.getResultant();
			uRealizedGlobal = self.uRealizedGlobal;
			
			self.integratePlant(dt);

		end


		function setState(self, x)
			if (length(x) == 13)
				% assume x contains vector quaternion
				% ...
			end
			qs = Quaternion.fromVec(x(7:9));
			X0 = reshape(x, length(x), 1);
			self.x = [X0(1:6); qs; X0(10:12)];
		end

	end

	methods (Access = private)

		function integratePlant(self, dt)
			self.x = RK4(@(x, dt) self.plantDynamics(x, dt), self.x, dt);
			self.x(7:10) = self.x(7:10) / norm(self.x(7:10));
			self.m = self.m - self.engine.specificThrust * self.uAllocated(1) * dt;
		end

		function dX = plantDynamics(self, x, dt)
			dX = [
				reshape(x(4:6), 3, 1)
				reshape(self.uRealizedGlobal(1:3) / self.m + [0 0 0]', 3, 1)
				reshape(1 / 2 * skew4(x(11:13)) * x(7:10), 4, 1)
				reshape(self.JInv * self.uRealizedGlobal(4:6), 3, 1)
			];
		end

		function getResultant(self)

			% Get vehicle frame forces
			uVehicleRealized = [
				self.engine.localizedResultant(1:3) + self.rcs.localizedResultant(1:3),
				cross([0 0 self.engineMountingDistance]', self.engine.localizedResultant(4:6)) + cross([0 0 -self.rcsMountingDistance]', self.rcs.localizedResultant(4:6))
			];

			self.uRealizedGlobal = [
				Quaternion.rotateBy(uVehicleRealized(1:3), self.x(7:10));
				uVehicleRealized(4:6)
			];

		end

		function uLocal = getLocalRequest(self)

			% Convert from body frame to 
			uLocal = [
				Quaternion.reverseRotateBy(self.uRequestedGlobal(1:3), self.x(7:10))
				self.uRequestedGlobal(4:6)
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

