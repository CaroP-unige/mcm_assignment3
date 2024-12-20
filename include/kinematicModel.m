%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            bTe = self.gm.getTransformWrtBase(self.gm.jointNumber);
            b_r_eb = bTe(1:3,4);
            jJb_A = zeros(3, self.gm.jointNumber);
            jJb_L = zeros(3, self.gm.jointNumber);

            for j = 1:self.gm.jointNumber
                bTj = self.gm.getTransformWrtBase(j);
                b_k_z = bTj(1:3,1:3)*[0; 0; 1];
                if ~self.gm.jointType(j)
                    jJb_A(:, j) = b_k_z;
                    
                    b_r_j = bTj(1:3,4);
                    j_r_e = b_r_eb - b_r_j;
                    jJb_L(:, j) = cross(b_k_z, j_r_e);
                    continue;
                end
                jJb_L(:, j) = b_k_z;
            end

            b_r_tb = self.gm.getToolTransformWrtBase();
            b_r_tb = b_r_tb(1:3,4);

            b_r_te = b_r_tb - b_r_eb;

            b_skew_r_e = [0 -b_r_te(3) b_r_te(2);
                b_r_te(3) 0 -b_r_te(1);
                -b_r_te(2) b_r_te(1) 0];

            b_S_e_b = [eye(3,3) zeros(3,3); b_skew_r_e' eye(3,3)];
            self.J = b_S_e_b * [jJb_A; jJb_L];
        end
    end
end