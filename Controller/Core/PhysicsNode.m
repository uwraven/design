classdef PhysicsNode < Node

properties (Access = public)
    mass = 1
    pressure = 0
    temperature = 273.15
    inertiaTensor = diag(ones(3,1))
    position = zeros(3, 1)
    orientation = diag(ones(3,1))
    renderable logical = false
end

methods (Access = public)
    function self = PhysicsNode(varargin)
    end

    function mass = totalMass(self)
        mass = self.mass;
        for i = 1:length(self.children)
            child = self.children(i);
            if class(child) == "PhysicsNode" || contains(superclasses(child), "PhysicsNode")
                mass = mass + child.totalMass();
            end
        end
    end

end

methods (Access = private)

end

end