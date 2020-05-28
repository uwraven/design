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

		% Hardware (Plants)
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
		uRealizedLocal
		uRealizedGlobal
	end
	

	methods (Access = public)

		function self = Vehicle()

			% Guidance and Control
			self.trajectoryPlanner = TrajectoryPlanner();
			self.controller = CascadedController();
			self.allocator = Allocator();
			self.estimator = Estimator();

			% Hardware
			self.engine = EngineAssembly();
			self.rcs = ReactionControl();

		end


		function update(self, dt)
			self.trajectoryPlanner.update(dt);
			self.controller.update(dt);
			self.allocator.update(dt);

			if (self.estimator.enabled)
				% TODO estimator
			end

			% Compute requested inputs from controller
			% Get global acceleration and angular acceleration from controller node
			uRequestedGlobalController = self.controller.inputs(self.x, self.trajectoryPlanner.filteredCoordinate, dt);

			% Add gravity feedforward to requested acceleration
			gravityFeedforward = -9.81;
			self.uRequestedGlobal = uRequestedGlobalController + [0 0 gravityFeedforward 0 0 0]';

			if (self.allocator.enabled)

				% Convert global acceleration request to vehicle fixed frame
				uRequestedLocal = self.getLocalRequest();

				% Convert requests to forces and torques
				uRequestedLocalForce = [
					self.m * uRequestedLocal(1:3)
					self.J * uRequestedLocal(4:6)
				];

				% Allocate inputs
				uAllocated = self.allocator.clampedLinearAllocation(uRequestedLocalForce);
				self.uAllocated = uAllocated;

				% Assign inputs to actuators
				self.engine.setAllocation(self.uAllocated(1:3));
				self.rcs.setAllocation(self.uAllocated(4:end));	
				
				self.engine.update(dt);
				self.rcs.update(dt);

				resultant = self.getRequestResultant();
				self.uRealizedGlobal = resultant;

			else
				% Integrate plants to maintain simulation continuity
				% self.engine.update(dt);
				% self.rcs.update(dt);
				% No allocation and no actuation, use pure and global controller outputs
				self.uRealizedGlobal = self.uRequestedGlobal * self.m;
			end
			
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
			self.trajectoryPlanner.setInitialTarget(self.x(1:3));
		end

	end

	methods (Access = private)

		function integratePlant(self, dt)
			self.x = RK4(@(x, dt) self.plantDynamics(x, dt), self.x, dt);
			self.x(7:10) = self.x(7:10) / norm(self.x(7:10));
			% self.m = self.m - self.engine.specificThrust * self.uAllocated(1) * dt;
		end

		function dX = plantDynamics(self, x, dt)
			dX = [
				x(4:6)
				self.uRealizedGlobal(1:3) / self.m + [0 0 9.81]'
				1 / 2 * skew4(x(11:13)) * x(7:10)
				self.JInv * self.uRealizedGlobal(4:6)
			];
		end

		function uResultant = getRequestResultant(self)

			% Get vehicle frame forces
			self.uRealizedLocal = [
				self.engine.localizedResultant(1:3) + self.rcs.localizedResultant(1:3),
				cross([0 0 self.engineMountingDistance]', self.engine.localizedResultant(4:6)) + cross([0 0 -self.rcsMountingDistance]', self.rcs.localizedResultant(4:6))
			];

			uResultant = [
				Quaternion.rotateBy(self.uRealizedLocal(1:3), self.x(7:10));
				self.uRealizedLocal(4:6)
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

