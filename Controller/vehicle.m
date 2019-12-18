classdef vehicle
	
	properties
		m0
		mp
		mwet
		engine
		controller
	end
	
	methods
		function self = vehicle(m0, mp, mwet, engine, controller)
			self.m0 = m0;
			self.mp = mp;
			self.mwet = mwet;
			self.engine = engine;
			self.controller = controller;
		end
	end
end

