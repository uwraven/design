classdef Vehicle < handle

	properties (Access = public)

		% Physical properties
		wet_mass
		dry_mass
		len
		J

		% Vehicle state is composed of [r v q w]
		% For 2D, state is [x dx y dy th dth]
		state 

		% Outer loop controllers
		controllers

		% Actuators
		rcs ReactionControl
		
	end

	properties (Access = private)
		commandFunction
	end
	
	methods (Access = public)
		function self = Vehicle(mwet, mdry, len)
			self.wet_mass = mwet;
			self.dry_mass = mdry;
			self.len = len;
			self.J = 1 / 12 * len * wet_mass;

			self.controllers = struct(...
				'x', PIDController(0, 0, 0),...
				'y', PIDController(0, 0, 0),...
				'theta', PIDController(0, 0, 0));
		end

		function update(self, dt)
			
		end

		function setReferenceStateHandle(self, commandFunction)
			self.commandFunction = commandFunction;
		end
	
	end
end

