classdef PhysicsNode < Node

properties (Access = public)
    mass = 1
    pressure = 0
    temperature = 273.15
    inertiaTensor = diag(ones(3,1))
    position = zeros(3, 1)
    orientation = diag(ones(3,1))
end

methods (Access = public)
    function self = PhysicsNode(varargin)
    end
end

methods (Access = private)

end

end