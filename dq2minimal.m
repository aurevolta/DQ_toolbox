function [xiq,xit] = dq2minimal(a)
% DQ2MINIMAL Dual quaternion minimal representation
%
% [xiq,xit] = DQ2MINIMAL(a) computes the attitude minimal representation 
% vector [xiq] of dimensions [3x1] and the minimal position vector 
% representation [xit] of dimensions [3x1] from the input dual quaternion 
% [a] of dimensions [8x1]
%
% if the input has dimensions [3xn] the output will be two [3xn] matrix
%
% see also MINIMAL2DQ, MINIMAL2Q, Q2MINIMAL

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% compute the sizes
[m,n]=size(a);

% check dimension
if m ~= 8
    error 'input is not a dual quaternion'
end

if n == 1
    % extract euler angle
    th = 2 * acos(a(4));
    
    % determine euler axis
    if th ~= 0
        e = a(1:3,1)/sin(th/2);
    else
        e = [0;0;0];
    end
    
    % determine minimal representation of attitude quaternion (sign opposite)
    xiq = -e*th;
    
    % extract position
    d = dq2cartesian(a(:,1));
    
    % useful vector (t(1:3) deprived of the component along the euler axis)
    k = -.5*(crossm(e)*sin(th/2)+crossm(e)^2*cos(th/2))*d;
    
    % get the position quaternion minimal representation
    xit = 2*k/sinc(th/2/pi) + e*(e'*d);
    
else
    
    % initialize
    [xit,xiq]=deal(zeros(3,n));
    
    % recursively compute
    for i = 1 : n
        [xiq(:,i),xit(:,i)] = dq2minimal(a(:,i));
    end
end



end