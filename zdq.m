function zero_dual_quaternion = zdq(n)
% ZDQ Null dual quaternion 
%
% z = ZDQ generates a dual quaternion z with null attitude quaternion and
% null position quaternion. z = [0;0;0;1;0;0;0;0]
% Z = ZDQ(n) generates n dual quaternion z with null attitude quaternion and
% null position quaternion. Z = [z,z,...,z] and is [8xn]

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%
if nargin == 0
    zero_dual_quaternion = [0;0;0;1;0;0;0;0];
else
    zero_dual_quaternion = [zeros(3,n);ones(1,n);zeros(4,n)];
end

end