classdef Artva
    properties
        position
    end
    methods
        % Constructor
        function obj = Artva(p)
            if nargin == 1
                obj.position = p;
            end
        end

        function [phi, signal] = getSignal(obj, dronePos)
            droneX = dronePos(1);
            droneY = dronePos(2);
            droneZ = dronePos(3);

            artvaX = obj.position(1);
            artvaY = obj.position(2);
            artvaZ = obj.position(3);

            phi = [droneX^2,            ...
                   2 * droneX * droneY, ...
                   2 * droneX * droneZ, ...
                   droneY^2,            ...
                   2 * droneY * droneZ, ...
                   droneZ^2,            ...
                   -2 * droneX,         ...
                   -2 * droneY,         ...
                   -2 * droneZ,         ...
                   1];

            x = [1,      ...
                 0,      ...
                 0,      ...
                 1,      ...
                 0,      ...
                 1,      ...
                 artvaX, ...
                 artvaY, ...
                 artvaZ, ...
                 obj.position*obj.position.'];

            signal = phi * x.';
        end
    end
end