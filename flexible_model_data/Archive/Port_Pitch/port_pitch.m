clear
clc

addpath('/home/brad/bit-matlab-sim/Miscellaneous/')

err = 1
pp_p.one.locx = -406.789000;
pp_p.one.locy = 2181.096000;
pp_p.two.locx = -400.000000;
pp_p.two.locy = 2186.324000;

%vector along pitch to calculate rotation matrix
vp1 = [pp_p.two.locx - pp_p.one.locx; pp_p.two.locy - pp_p.one.locy]
ang = 37.5988150;
kp = 1;
ki = 0.01;
vp = [0;1];
sm = 0;

while norm(err) > 1e-18
    err = -vp(2);
    sm = sm + err;
    ang = ang - (kp * err) - (ki * sm);
    rot = axis2rot([0;0;1], deg2rad(ang));
    rot2 = rot(1:2,1:2);
    vp = rot2*vp1;
end

pp_p.thr.locx = -393.211000;
pp_p.thr.locy = 2191.552000;

vp1 = [pp_p.two.locx - pp_p.one.locx; pp_p.two.locy - pp_p.one.locy]
vp2 = [pp_p.thr.locx - pp_p.two.locx; pp_p.thr.locy - pp_p.two.locy]

pp_p.one.id =  1982150;
pp_p.two.id = 1982151;
pp_p.thr.id = 1989883;

pp_xp.one.id =  1989893;
pp_xp.two.id = 1982151;
pp_xp.thr.id = 1982144;

tab_x = readtable('port_pitch_wx.csv');
col1x = tab_x(:,1);
col2x = tab_x(:,2);
col3x = tab_x(:,3);

tab_y = readtable('port_pitch_wy.csv');
col1y = tab_y(:,1);
col2y = tab_y(:,2);
col3y = tab_y(:,3);

ind = height(col1x)/5;

set1x = table2array(col3x(1:ind, "Var3"));%XP2
set2x = table2array(col3x(ind+1:2*ind, "Var3"));%CENTER
set3x = table2array(col3x(2*ind+1:3*ind, "Var3"));%XP1
set4x = table2array(col3x(3*ind+1:4*ind, "Var3"));%P1
set5x = table2array(col3x(4*ind+1:5*ind, "Var3"));%P2

%Same order as X sets
set1y = table2array(col3y(1:ind, "Var3"));
set2y = table2array(col3y(ind+1:2*ind, "Var3"));
set3y = table2array(col3y(2*ind+1:3*ind, "Var3"));
set4y = table2array(col3y(3*ind+1:4*ind, "Var3"));
set5y = table2array(col3y(4*ind+1:5*ind, "Var3"));

w_xp_xy1 = [set3x'; set3y']; 
w_xp_xy2 = [set1x'; set1y']; 
w_c_xy1 = [set2x'; set2y']; 
w_p_xy1 = [set4x'; set4y']; 
w_p_xy2 = [set5x'; set5y'];

%rotate the deformation into pitch and xp
w_xp_pxp1 = rot2*w_xp_xy1;
w_xp_pxp2 = rot2*w_xp_xy2;
w_p_pxp1 = rot2*w_p_xy1;
w_p_pxp2 = rot2*w_p_xy2;
w_c_pxp1 = rot2*w_c_xy1;

%% other axis
pp_xp.one.locx = -394.772000;
pp_xp.one.locy = 2179.535000;
pp_xp.two.locx = -400.000000;
pp_xp.two.locy = 2186.324000;
pp_xp.thr.locx = -405.228000;
pp_xp.thr.locy = 2193.113000;

% 
vxp_one = [pp_xp.two.locx - pp_xp.one.locx; pp_xp.two.locy - pp_xp.one.locy]
vxp_two = [pp_xp.thr.locx - pp_xp.two.locx; pp_xp.thr.locy - pp_xp.two.locy]

% check rotation gives only change along xp
vxp1 = rot2 * vxp_one
vxp2 = rot2 * vxp_two

% assemple mode shape deformation in array where col 1 is point 1 and col 3
% is point 3 along pitch axis and then xp axis
p_wxp = [w_p_pxp1(2,:)',w_c_pxp1(2,:)', w_p_pxp2(2,:)']

xp_wp = [w_xp_pxp1(1,:)',w_c_pxp1(1,:)', w_xp_pxp2(1,:)']

% calculate partial of deofrmation in xp wrt p
deriv_p_wxp = diff(p_wxp,1,2);
scaled_diff_p_wxp = [deriv_p_wxp(:,1)/vp1(1), deriv_p_wxp(:,2)/vp2(1)];

% calculate partial of deofrmation in p wrt xp
deriv_xp_wp = diff(xp_wp,1,2);
scaled_diff_xp_wp = [deriv_xp_wp(:,1)/vxp1(2), deriv_xp_wp(:,2)/vxp2(2)];

contrib_xp = mean(scaled_diff_xp_wp,2)
contrib_p = mean(scaled_diff_p_wxp ,2)





