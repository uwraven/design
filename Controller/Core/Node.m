classdef Node < handle

properties (Access = public)
    name string
    parent Node
    children = []
    recordable logical = true
    recordableVariables = []
    timer Timer
end

methods (Access = public)
    function self = Node(varargin)
        if length(varargin) >= 1
            self.name = varargin{1};
        end
        if length(varargin) >= 2
            self.recordable = varargin{2};
        end
        if length(varargin) >= 3
            self.recordableVariables = varargin{3};
        end
        self.timer = Timer();
        self.timer.setLoopMethodHandle(@self.onLoop);
    end

    function addChild(self, node)
        node.parent = self;
        self.children = [self.children, node];
    end

    function addChildren(self, nodes)
        for i = 1:length(nodes)
            nodes(i).parent = self;
        end
        self.children = [self.children, nodes];
    end

    function [node, ind] = childWithName(self, name)
        % returns first child node with specified name
        for i = 1:length(self.children)
            if self.children(i).name == name
                node = self.children(i);
                ind = i;
                return;
            end
        end
        node = false;
    end

    function nodes = childrenWithName(self, name)
        % returns all child nodes with specified name
        nodes = [];
        for i = 1:length(self.children)
            if (self.children(i).name == name)
                nodes = [nodes, self.children(i)];
            end
        end
    end

    function removeChildren(self)
        self.children = [];
    end

    function arr = traverseRecordableVariableValues(self, includeSelf)
        arr = [];

        % loops through all children
        if ~self.recordable
            warning("Node instance is not recordable, returning []");
            return
        end

        if isempty(varargin)
            includeSelf = false;
        end

        % iterate over this node's recordable vars only if start of recursion
        if includeSelf
            for i = 1:length(self.recordableVariables)
                arr = [arr self.rowReshape(self.recordableVariables(i))];
            end
        end

        % loop through child tree and aggregate all child vars
        % reshape to form a single row vector
        for i = 1:length(self.children)
            if (self.children(i).recordable)
                child = self.children(i);
                for j = 1:length(child.recordableVariables)
                    varName = child.recordableVariables(j);
                    arr = [arr, self.rowReshape(self.(varName))];
                end
                arr = [arr, child.traverseRecordableVariableValues(false)];
            end
        end
    end

    function names = traverseRecordableVariableNames(self, includeSelf)
        names = [];
        if includeSelf
            names = self.reshapeRecordableVariables(self);
        end
        % recurse through child tree and get all variable names, reshape 
        % and fill for multidimensional vars
        for i = 1:length(self.children)
            if (self.children(i).recordable)
                child = self.children(i);
                names = [names self.reshapeRecordableVariables(child)];
                names = [names child.traverseRecordableVariableNames(false)];
            end
        end
    end

    function onLoop(self, varargin)
    end

end

methods (Static)

    % Static helper method to generate variable names for large vectors
    % and arrays, ensure that all columns in a row are matched 1:1 
    % to var names
    function names = reshapeRecordableVariables(node)
        names = [];
        node.name
        for j = 1:length(node.recordableVariables)
            var = node.recordableVariables(j);
            if length(node.(var)) > 1
                sz = size(node.(var));
                newNames = strings(sz);
                for rr = 1:sz(1)
                    % traverse columns
                    for cc = 1:sz(2)
                        % for each index create a unique var name 
                        newNames(rr, cc) = strcat(string(rr), "_", string(cc));
                    end
                end
                newNames = append(strcat(var, "_"), newNames);
                names = [names, rowReshape(newNames)];
            else
                names = [names, node.recordableVariables(j)];
            end
        end
    end

    function arr = rowReshape(arr)
        arr = reshape(arr, 1, []);
    end

end

end