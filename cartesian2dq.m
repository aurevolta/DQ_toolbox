function [varargout]=cartesian2dq(varargin)
% CARTESIAN2DQ Conversion from cartesian variables to dual quaternions
% 
% a = CARTESIAN2DQ(x,R) computes the dual quaternion [a] of dimensions
% [8x1] from the input position vector [x] of dimensions [3x1] and the DCM
% [R] of dimensions [3x3]. 
%
% A = CARTESIAN2DQ(x,R) computes the dual quaternion [a] of dimensions
% [8xn] from the input position vector [x] of dimensions [3xn] and the DCM
% [R] of dimensions [3x3xn]. 
% 
% [q,t] = CARTESIAN2DQ(x,R) behaves like CARTESIAN2DQ(x,R) but the output
% is split between rotation quaternion [q] and translation quaternion [t]
% such that [a] = [[q];[t]]
%
% a = CARTESIAN2DQ(x,q) behaves like CARTESIAN2DQ(x,R) but the attitude is
% already expressed as quaternion. 
% 
% [a,da] = CARTESIAN2DQ(x,v,omega,R) computes the dual quaternion [a] and
% its derivative [da], both of dimensions [8x1], from the input position
% vector [x], linear velocity [v] and angular velocity (inertial) [omega] all of
% dimensions [3x1]. [R] is the input DCM of dimensions [3x3]. It can be
% substituted by a [4x1] quaternion like CARTESIAN2DQ(x,q).  
% 
% [a,da] = CARTESIAN2DQ(x,v,omega,R) supports multiple, consistent inputs.
% [a] and [da] will have dimensions [8xn] where [n] is the number of
% columns in [x],[v] and [omega]. Dimensions of [R] shall be [3x3xn] or
% [4xn] if the input is a quaternion.
% 
% see also DQ2CARTESIAN, Q2DCM, DCM2Q

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

if nargin == 2
    % convert position and attitude into a quaternion
    
    % extract the input
    x = varargin{1};
    b = varargin{2};
    
    % check dimensions
    [m,n,z] = size(b);
    
    if m == n && m == 3
        % the input is a DCM
        g = z;
        q = DCM2q(b);
    else
        % the input is a quaternion
        g = n;
        q = quat_normalize(b); % normalize just to be sure
    end
    
    if size(x,2) ~= g
        error 'non consistent input dimensions'
    end
    
    % initialize
    t = zeros(4,g);
    
    % compute
    for i = 1 : g
        t(:,i) = .5*crossqp([x(:,i);0])*q(:,i);
    end
    
    % outputs
    if nargout == 1    
        varargout{1} = [q;t];    
    elseif nargout == 2
        varargout{1} = q;
        varargout{2} = t;
    else
        error 'wrong number of outputs'
    end
    
else
    % the input consists also on velocities
    
    x = varargin{1};
    v = varargin{2};
    omega = varargin{3};
    b = varargin{4};
    
    % check dimensions
    [m,n,z] = size(b);
    
    if m == n && m == 3
        % the input is a DCM
        g = z;
        q = DCM2q(b);
    else
        % the input is a quaternion
        g = n;
        q = quat_normalize(b); % normalize just to be sure
    end
    
    if size(x,2) ~= g || size(v,2) ~= g || size(omega,2) ~= g
        error 'non consistent input dimensions'
    end
    
    % initialize
    [t,qdot,tdot] = deal(zeros(4,g));
    
    % loop the conversion
    for i = 1 : g
        t(:,i) = .5*crossqp([x(:,i);0])*q(:,i);
        qdot(:,i) = .5*crossqp([omega(:,i);0])*q(:,i);
        tdot(:,i) = .5*crossqm(q(:,i))*([v(:,i);0]-2*crossqmt(t(:,i))*qdot(:,i));
    end
    
    % prepare output
    if nargout == 2
        varargout{1} = [q;t];
        varargout{2} = [qdot;tdot];
    else
        varargout{1} = qdot;
        varargout{2} = tdot;
        varargout{3} = q;
        varargout{4} = t;
    end
    
end

end