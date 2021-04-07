classdef Force 
    properties
        X,
        Y,
        Z,
        L {mustBeNumeric},
        R {mustBeNumeric},
        Theta {mustBeNumeric}
    end
    methods
        function obj = Force(X, Y, Z, L, R, Theta)
            if nargin == 4
                R = 0;
                Theta = 0;
            elseif nargin == 5
                Theta = 0;
            end
            
            obj.X = X;
            obj.Y = Y;
            obj.Z = Z;
            obj.L = L;
            obj.R = R;
            obj.Theta = Theta;
        end
    end
end