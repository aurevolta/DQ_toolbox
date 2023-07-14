function [varargout]=dq2cartesian(varargin)
% DQ2CARTESIAN Conversion from dual quaternion to cartesian variables
%
% [x,R] = DQ2CARTESIAN(a) % convert the dual quaternion [a] of
% dimensions [8x1] into its correspective position vector [x] and DCM [R].  
% 
% [x,v,omega,q,R,omegab] = DQ2CARTESIAN(a,da) convert the dual quaternion
% [a] and its derivative [da] from [8x1] vectors to the cartesian
% equivalent position vecotr [x], velocity vector [v], angular velocity
% [omega], quaternion [q], DCM [R] and angular velocity in the other system
% [omegab]. 
% 
% [x,v,omega,q,R,omegab] = DQ2CARTESIAN(q,t,dq,dt) is equivalent to 
% DQ2CARTESIAN(a,da) where [a] = [[q];[t]] and [da] = [[dq];[dt]]
% 
% The function supports multiple entries, if [a] is [8xn] the outputs will
% have second dimension of length n except the DCM that will be [3x3xn].
% 
% see also Q2DCM, DCM2Q, CARTESIAN2DQ

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%
if nargin == 1
    % the input is only position and attitude dual quaternion. 
    
    % extract the two quaternions
    q=varargin{1}(1:4,:);
    t=varargin{1}(5:8,:);

    % determines the size of the input
    [~,n] = size(q);
    
    % initialize
    X = zeros(4,n,'like',q);
    R = zeros(3,3,n,'like',q);
    
    % conversion loop
    for i=1:n
        X(:,i) = 2*crossqm(q(:,i))'*t(:,i);
        R(:,:,i) = q2dcm(q(:,i));
    end
    
    % truncate
    x = X(1:3,:);
    
    % collect and assign to output
    varargout{1} = x;
    varargout{2} = R;
    
    return

elseif nargin == 2
    % process the [a,da] input
    qdot = varargin{2}(1:4,:);
    tdot = varargin{2}(5:8,:);
    q = varargin{1}(1:4,:);
    t = varargin{1}(5:8,:);
    
elseif nargin == 4
    % process the [q,t,dq,dt] input
    qdot = varargin{1};
    tdot = varargin{2};
    q = varargin{3};
    t = varargin{4};  
end

% determine number of instances to be translated
[~,n] = size(q);

% initialize
[x,v,omega,omegab] = deal(zeros(4,n,'like',q));
R = zeros(3,3,n,'like',q);

% conversion loop
for i=1:n
    x(:,i) = 2*crossqm(q(:,i))'*t(:,i);
    v(:,i) = 2*crossqm(q(:,i))'*tdot(:,i)+2*crossqmt(t(:,i))*qdot(:,i);
    omega(:,i) = 2*crossqm(q(:,i))'*qdot(:,i);
    R(:,:,i)=q2dcm(q(:,i));
    omegab(:,i) = 2*crossqp(q(:,i))'*qdot(:,i);
end


% truncate
x = x(1:3,:);
v = v(1:3,:);
omega = omega(1:3,:);
omegab = omegab(1:3,:);

% collect output
varargout{1} = x;
varargout{2} = v;
varargout{3} = omega;
varargout{4} = q;
varargout{5} = R;
varargout{6} = omegab;


end