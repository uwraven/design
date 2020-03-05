classdef Vehicle < handle

	properties (Access = public)
		% Unused cascading controller blocks
		% control_pos
		% control_att
		% control_rate

		m
		cf
		J
		JInv

		% LQR Controller and allocator
		control_lqr
		allocator
	end

	properties (GetAccess = public, SetAccess = private)
		% x is the state vector containing global position, velocity, error quaternion, and body frame angular rates
		% uRequested is the input vector containing global forces and body frame moments
		% uAllocated is the computed allocation using a linear or nonlinear strategy
		% uGlobalRealized is the realized global frame forces and body frame moments resulting from converted actuator commands, this is used to perform integration
		% R is the reference state: position, velocity, stability frame quaternion, body frame angular rates
		x
		uAllocated
		uRequested
		uGlobalRealized
		R
	end

	% properties (Dependent)
	% 	JInv;
	% end

	properties (Access = private)

	end
	
	methods (Access = public)
		function self = Vehicle()
			% TODO: Convert to varargin
			self.control_lqr = LQRController();
			self.allocator = Allocator();
		end

		function tick(self, dt)
			% get state errors, including error quaternion
			XR = self.R - [self.x(1:6); self.x(8:13)];
			qs = Quaternion.fromVec(self.R(7:9));
			qe = Quaternion.error(qs, self.x(7:10));
			XR(7:9) = qe(2:4);

			% Get inputs from LQR control block
			self.uRequested = self.control_lqr.inputs(XR);
			self.uRequested(1) = self.uRequested(1) + 9.81 * self.m;

			% Convert requested inputs to body frame
			uRequestedLocal = [
				Quaternion.rotateBy(self.uRequested(1:3), self.x(7:10))
				self.uRequested(4:6)
			];

			% Allocate inputs
			self.uAllocated = self.allocator.linearAllocation(uRequestedLocal);

			% Realize actuator inputs
			self.uGlobalRealized = self.recoverAllocatedInputs();
			
			self.integrateState(dt);
		end

		function setReference(self, R)
			self.R = reshape(R, length(R), 1);
		end

		function setState(self, x)
			qs = Quaternion.fromVec(x(7:9));
			X0 = reshape(x, length(x), 1);
			self.x = [X0(1:6); qs; X0(10:12)];
		end

		function setLQRGains(self, K)
			self.control_lqr.setGains(K);
		end

	end

	methods (Access = private)
		function integrateState(self, dt)
			self.x = RK4(@(x, dt) self.stateTransitionFunc(x, dt), self.x, dt);
			self.x(7:10) = self.x(7:10) / norm(self.x(7:10));
			self.m = self.m - self.cf * self.uAllocated(1) * dt;
		end

		function dX = stateTransitionFunc(self, x, dt)
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

