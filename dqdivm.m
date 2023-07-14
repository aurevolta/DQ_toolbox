function A = dqdivm(a)
% DQDIVM Dual quaternion division matrix (-)
%
%  A = DQDIVM(a) computes the dual quaternion division matrix form of a [8x1]
% if a is a matrix [8xn] the output A will be [8x8xn]. 
% Supports also symbolic variables. 
%
% see also DQDIVMT, DQDIVP, DQDIVPT, DQPRODM, DQPRODMT, DQPRODP, DQPRODPT.

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% compute the numbers of dual quaternions in input
[~,n] = size(a);

% generates the matrix 
A = zeros(8,8,n,'like',a);

% extract the dual quaternion rotation and translation parts
q = a(1:4,:);
t = a(5:8,:);

% compute the two submatrices (with multidimensional transpose)
QP = permute(crossqm(q),[2 1 3]);
TP = permute(crossqm(t),[2 1 3]);

% assign 
A(1:4,1:4,:) = QP;
A(5:8,5:8,:) = QP;
A(5:8,1:4,:) = TP;

end

