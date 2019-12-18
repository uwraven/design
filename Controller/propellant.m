classdef propellant
	%PROPELLANT Summary of this class goes here
	%   Detailed explanation goes here
	
	properties
		tmp
		dns
		vsc
		cnd
		cp
	end
	
	methods
		function self = propellant(dns, vsc, cnd, cp)
			self.tmp = 293.15;
			self.dns = dns;
			self.vsc = vsc;
			self.cnd = cnd;
			self.cp = cp;
		end
	end
end

