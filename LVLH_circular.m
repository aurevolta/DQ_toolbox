function [r,dr]=LVLH_circular(mu,radius,RAAN,inclination,theta0,t,theta)
% LVLH_CIRCULAR LVLH frame dual quaternion
%
%[r,dr] = LVLH_CIRCULAR(mu,radius,RAAN,inclination,theta0,t)
% generates dual quaternion [r] and dual quaternion velocity [dr] of the
% LVLH frame of a circular orbit (null eccentricity) given the gravity
% constant [mu], the radius [radius], the Right Ascension of the Ascending
% Node [RAAN], the inclination [inclination] and the initial true anomaly
% [theta0]. [t] is the time that identifies the orbital position making use
% of the angular velocity of the orbit.
%
% [r,dr] = LVLH_CIRCULAR(mu,radius,RAAN,inclination,theta0,[],theta) uses
% the true anomaly variation [theta] from [theta0] instead of time [t].

% Reference notes: The x axis of this frame is directed as pointing radius, 
% the vertical as the angular momentum vector of the circular orbit. 
% Consequently y is directed as the velocity.

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% angular velocity
n = sqrt (mu/(radius^3));

% initial attitude of the lvlh frame (using q_gen(axis,angle))
% q0 = crossqm([0;0;sin(theta0/2);cos(theta0/2)])*crossqm([sin(inclination/2);0;0;cos(inclination/2)])*[0;0;sin(RAAN/2);cos(RAAN/2)];
q0 = crossqm(q_gen('z',theta0))*crossqm(q_gen('x',inclination))*q_gen('z',RAAN);

% solution at time t / at anomaly theta
if nargin == 6
    S = sin(n/2*t);
    C = cos(n/2*t);
elseif nargin == 7
    S = sin(theta);
    C = cos(theta);
end

% get both attitude and its derivative
q = S*crossqm([0;0;1;0])*q0+C*q0;
dq = n/2*C*crossqm([0;0;1;0])*q0-n/2*S*q0;

% compose to get the dq
QD = crossqm([radius/2;0;0;0]);
r = [q;QD*q];
dr = [dq;QD*dq];

end