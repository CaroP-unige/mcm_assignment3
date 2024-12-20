%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    % eTt (OPTIONAL) add a tool to the model rigid attached to the
    % end-effector
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
        eTt
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType,eTt)
            if nargin > 1
                 if ~exist('eTt','var')
                     % third parameter does not exist, so default it to something
                      eTt = eye(4);
                 end
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
                self.eTt = eTt;
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            self.q = q;
        
            % Ciclo su ogni giunto per calcolare la trasformazione aggiornata.
            for j = 1:self.jointNumber
                % Trasformation matrix for q = 0.
                T_0 = self.iTj_0(:,:,j);
                T_joint = eye(4);

                switch self.jointType(j)
                    case 0 % Revolut
                        R_z = [cos(q(j)), -sin(q(j)), 0;
                               sin(q(j)),  cos(q(j)), 0;
                                     0,          0, 1];
                        T_joint(1:3, 1:3) = R_z;
        
                    case 1 % Giunto prismatico.
                        % Matrice di traslazione lungo z.
                        T_joint(3, 4) = q(j);

                    otherwise
                        error("Not valid values for q.");
                end
        
                self.iTj(:,:,j) = T_0 * T_joint;
            end
        end
        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            bTk = eye(4,4);
            for j=1:k
                bTk = bTk * self.iTj(:,:,j);
            end
        end
        function [bTt] = getToolTransformWrtBase(self)
            %% getToolTransformWrtBase function
            % Inputs :
            % None 
            % bTt : transformation matrix from the manipulator base to the
            % tool
            
            bTe = self.getTransformWrtBase(self.jointNumber);
            bTt = bTe * self.eTt;
        end
    end
end