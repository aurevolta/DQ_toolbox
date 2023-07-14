% Dual Quaternion Toolbox
% This toolbox contains function useful to deal with dual quaternions for
% attitude and position (kinematics) 
%
% Version 1.0 23-05-2017
%
% Division Matrix forms
%   dqdivm        - Dual quaternion division matrix (-)
%   dqdivmt       - Dual quaternion division matrix (-T)
%   dqdivp        - Dual quaternion division matrix (+)
%   dqdivpt       - Dual quaternion division matrix (+T)
%
% Product Matrix forms
%   dqprodm       - Dual quaternion product matrix (-)
%   dqprodmt      - Dual quaternion product matrix (-T)
%   dqprodp       - Dual quaternion product matrix (+)
%   dqprodpt      - Dual quaternion product matrix (+T)
%
% Conversions
%   cartesian2dq  - Conversion from cartesian variables to dual quaternions
%   dq2cartesian  - Conversion from dual quaternion to cartesian variables
%   dq_conj       - Dual quaternion conjugate
%   dq2minimal    - Dual quaternion minimal representation
%   minimal2dq    - Dual quaternion conversion from minimal representation
%
% Dual quaternion generations
%   randdq        - Random dual quaternion
%   zdq           - Null dual quaternion 
%   LVLH_circular - LVLH frame dual quaternion


