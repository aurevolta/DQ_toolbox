function a = randdq(D)
%RANDDQ Random dual quaternion
%
%  a = RANDDQ() generates a random dual quaternion with rectangular
%  distribution position vector (-1,1). 
%  a = RANDDQ(D) generates a random dual quaternion with rectangular
%  distribution position vector (-D,D). 
%
% see also RANDQ.

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta
%% random dual quaternion

if nargin == 0
    D = 1;
end

% generates random unitary quaternion
q = randq;
% generates the random position quaternion
t = crossqm(q)*[2*(rand(3,1)-.5)*D;0];
% collect together for output
a = [q;t];

end

