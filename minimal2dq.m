function a = minimal2dq(xiq,xit)
% MINIMAL2DQ Dual quaternion conversion from minimal representation
%
% a = MINIMAL2DQ(xiq,xit) computes the dual quaternion [a] from the
% attitude minimal representation vector [xiq] of dimensions [3x1] and the
% minimal position vector representation [xit] of dimensions [3x1]
%
% if the inputs have dimensions [3xn] the output will be [8xn] matrix
%
% see also DQ2MINIMAL, MINIMAL2Q, Q2MINIMAL

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% compute the sizes
[m,n] = size(xiq);
[mt,nt] = size(xiq);

% check dimension
if m ~= 3 || m ~= mt || n ~= nt
    error 'input is not the minimal representation of a dual quaternion'
end

if n == 1
    % extract euler angle
    th = norm(xiq);
    
    % determine euler axis
    if th ~= 0
        e = -xiq/th;
    else
        e = [0;0;0];
    end
    
    % determine attitude quaternion
    q = [e*sin(th/2);cos(th/2)];
    
    % generate matrix such that xit = M * d
%     M = -1/sinc(th/2/pi)*(crossm(e)*sin(th/2)+cos(th/2)*crossm(e)^2)+e*(e');
    M = -th/2*(crossm(e)+1/tan(th/2)*crossm(e)^2)+e*(e');

    % compute position
    d = M\xit;
    
    % determine position quaternion
    t = .5 * crossqm(q)*[d;0];
    
    % assemble for output
    a = [q;t];
    
else
    
    % initialize
    a=zeros(8,n);
    
    % recursively compute
    for i = 1 : n
        [a(:,i)] = minimal2dq(xiq(:,i),xit(:,i));
    end
    
end



end