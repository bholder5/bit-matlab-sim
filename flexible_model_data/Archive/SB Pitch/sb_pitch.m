clear
clc

addpath('/home/brad/bit-matlab-sim/Miscellaneous/')

err = 1
sb_p_p.one.locx = -406.789000;
sb_p_p.one.locy = 2177.482000;
sb_p_p.two.locx = -400.000000;
sb_p_p.two.locy = 2182.710000;

%vector along pitch to calculate rotation matrix
vp1 = [sb_p_p.two.locx - sb_p_p.one.locx; sb_p_p.two.locy - sb_p_p.one.locy]
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
    vp = rot2*vp1
end

sb_p_p.thr.locx = -393.211000;
sb_p_p.thr.locy = 2187.938000;

vp1 = [sb_p_p.two.locx - sb_p_p.one.locx; sb_p_p.two.locy - sb_p_p.one.locy]
vp2 = [sb_p_p.thr.locx - sb_p_p.two.locx; sb_p_p.thr.locy - sb_p_p.two.locy]

sb_p_p.one.id =  1982150;
sb_p_p.two.id = 1982151;
sb_p_p.thr.id = 1989883;

sb_p_xp.one.id =  1989893;
sb_p_xp.two.id = 1982151;
sb_p_xp.thr.id = 1982144;

tab_x = readtable('sb_pitch_wx.csv');
col1x = tab_x(:,1);
col2x = tab_x(:,2);
col3x = tab_x(:,3);

tab_y = readtable('sb_pitch_wy.csv');
col1y = tab_y(:,1);
col2y = tab_y(:,2);
col3y = tab_y(:,3);

ind = height(col1x)/5;

set1x = table2array(col3x(1:ind, "Var3"));%XP1
set2x = table2array(col3x(ind+1:2*ind, "Var3"));%XP2
set3x = table2array(col3x(2*ind+1:3*ind, "Var3"));%CENTER
set4x = table2array(col3x(3*ind+1:4*ind, "Var3"));%P2
set5x = table2array(col3x(4*ind+1:5*ind, "Var3"));%P1

%Same order as X sets
set1y = table2array(col3y(1:ind, "Var3"));
set2y = table2array(col3y(ind+1:2*ind, "Var3"));
set3y = table2array(col3y(2*ind+1:3*ind, "Var3"));
set4y = table2array(col3y(3*ind+1:4*ind, "Var3"));
set5y = table2array(col3y(4*ind+1:5*ind, "Var3"));

w_xp_xy1 = [set1x'; set1y']; 
w_xp_xy2 = [set2x'; set2y']; 
w_c_xy1 = [set3x'; set3y']; 
w_p_xy1 = [set5x'; set5y']; 
w_p_xy2 = [set4x'; set4y'];

%rotate the deformation into pitch and xp
w_xp_pxp1 = rot2*w_xp_xy1;
w_xp_pxp2 = rot2*w_xp_xy2;
w_p_pxp1 = rot2*w_p_xy1;
w_p_pxp2 = rot2*w_p_xy2;
w_c_pxp1 = rot2*w_c_xy1;

%% other axis
sb_p_xp.one.locx = -394.772000;
sb_p_xp.one.locy = 2175.921000;
sb_p_xp.two.locx = -400.000000;
sb_p_xp.two.locy = 2182.710000;
sb_p_xp.thr.locx = -405.228000;
sb_p_xp.thr.locy = 2189.499000;

% 
vxp_one = [sb_p_xp.two.locx - sb_p_xp.one.locx; sb_p_xp.two.locy - sb_p_xp.one.locy]
vxp_two = [sb_p_xp.thr.locx - sb_p_xp.two.locx; sb_p_xp.thr.locy - sb_p_xp.two.locy]

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





