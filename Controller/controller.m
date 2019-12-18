classdef controller
	%CONTROLLER Summary of this class goes here
	%   Detailed explanation goes here
	
	properties
		K
		xa
		xd
		u
	end
	
	methods
		function self = controller(K)
			self.K = K;
		end
	end
end

