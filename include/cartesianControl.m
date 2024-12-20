%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            b_ni_tb = [self.k_a * eye(3) zeros(3); zeros(3) self.k_l * eye(3)];
            bTt = self.gm.getToolTransformWrtBase();
            tTg = bTt\bTg;

            R = tTg(1:3,1:3);

            I = eye(3,3);
            % Check matrix R to see if its size is 3x3
            if any([3 3] ~= size(R))
                error("R is not a 3x3 matrix.")
            end
            if ((det(R) - 1 > 1e-10) || any(abs(R * R' - I) > 1e-10,"all"))
                error("R can't be a rotation matrix.")
            end
        
            theta = acos((sum(R .* I, "all") - 1) / 2);
        
            if (theta == 0)
                error("The angle of rotation is zero, h can be arbitrary.")
            elseif abs(theta - pi) < 1e-10
                h_pos = sqrt((diag(R) + 1) / 2);
        
                % max() make found the not null component
                [~, idx] = max(abs(h_pos));
                
                % Set the sgn of ather components with repect R's compoents
                if idx == 1
                    if R(1, 2) < 0
                        h_pos(2) = -abs(h_pos(2));
                    end
                    if R(1, 3) < 0
                        h_pos(3) = -abs(h_pos(3));
                    end
                elseif idx == 2
                    if R(1, 2) < 0
                        h_pos(1) = -abs(h_pos(1));
                    end
                    if R(2, 3) < 0
                        h_pos(3) = -abs(h_pos(3));
                    end
                else
                    if R(1, 3) < 0
                        h_pos(1) = -abs(h_pos(1));
                    end
                    if R(2, 3) < 0
                        h_pos(2) = -abs(h_pos(2));
                    end
                end
        
                h_pos = h_pos / norm(h_pos);
                h_neg = -h_pos; % make the second solution equal and opposite
                disp(h_pos)
                h = [h_pos, h_neg];
            else
                S_a = (R - R') / 2;
                a = [S_a(3, 2); S_a(1, 3); S_a(2, 1)];
                % Axial vector a, a = sin(theta) * h => h = a / sin(theta)
                h = a / sin(theta);
            end

            b_rho_tg = bTt(1:3,1:3) * h * theta;

            b_r_tg = bTg(1:3,4) - bTt(1:3,4);

            x_dot = b_ni_tb * [b_rho_tg; b_r_tg];
        end
    end
end