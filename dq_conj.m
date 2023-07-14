function ac = dq_conj(a)
% DQ_CONJ Dual quaternion conjugate
% This function outputs the conjugate of the input dual quaternions
%
% ac = DQ_CONJ(a) generates the conjugate of the dual quaternion a [8x1]
%
% Ac = DQ_CONJ(A) generates the conjugate of the dual quaternions contained
% in A [8xn]
%
% see also QCONJ

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

ac = [-a(1:3,:)
       a(4,:)
      -a(5:7,:)
       a(8,:)];

end