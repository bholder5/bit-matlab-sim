function crl = sb_pitch()
    clear
    clc
    
    addpath('/home/brad/bit-matlab-sim/Miscellaneous/')
    % addpath('/home/brad/bit-matlab-sim/flexible_model_data/pitch_port')
    
    err = 1
    pp_p.one.locx = -406.789000;
    pp_p.one.locy = 2177.482000;
    pp_p.two.locx = -400.000000;
    pp_p.two.locy = 2182.71000;
    pp_p.thr.locx = -393.211000;
    pp_p.thr.locy = 2187.938000;
    
    %% other axis
    pp_xp.one.locx = -394.772000;
    pp_xp.one.locy = 2175.921000;
    pp_xp.two.locx = -400.000000;
    pp_xp.two.locy = 2182.710000;
    pp_xp.thr.locx = -405.228000;
    pp_xp.thr.locy = 2189.499000;
    
    %vector along pitch to calculate rotation matrix
    vb1 = [pp_p.two.locx - pp_p.one.locx; pp_p.two.locy - pp_p.one.locy];
    vb2 = [pp_p.thr.locx - pp_p.two.locx; pp_p.thr.locy - pp_p.two.locy];
    vxp1 = [pp_xp.two.locx - pp_xp.one.locx; pp_xp.two.locy - pp_xp.one.locy];
    vxp2 = [pp_xp.thr.locx - pp_xp.two.locx; pp_xp.thr.locy - pp_xp.two.locy];
     
    dotprod = vb1' * [1;0];
    dotprod = dotprod/norm(vb1);
    ang = acos(dotprod);
    ang1 = rad2deg(ang);
    
    dotprod2 = vb2' * [1;0];
    dotprod2 = dotprod2/norm(vb2);
    ang2 = acos(dotprod2);
    ang2 = rad2deg(ang2);
    
    dotprod3 = vxp1' * [0;1];
    dotprod3 = dotprod3/norm(vxp1);
    ang3 = acos(dotprod3);
    ang3 = rad2deg(ang3);
    
    dotprod4 = vxp2' * [0;1];
    dotprod4 = dotprod4/norm(vxp2);
    ang4 = acos(dotprod4);
    ang4 = rad2deg(ang4);
    
    ang = mean([ang1, ang2, ang3, ang4])
    
    rot = axis2rot([0;0;1], deg2rad(ang));
    rot2 = rot(1:2,1:2);
    
    vb1 = rot2 * vb1;
    vb2 = rot2 * vb2;
    
    tab_x = readtable('/home/brad/bit-matlab-sim/flexible_model_data/pitch_sb/wx.csv');
    col1x = tab_x(:,1);
    col2x = tab_x(:,2);
    col3x = tab_x(:,3);
    
    tab_y = readtable('/home/brad/bit-matlab-sim/flexible_model_data/pitch_sb/wy.csv');
    col1y = tab_y(:,1);
    col2y = tab_y(:,2);
    col3y = tab_y(:,3);
    
    ind = height(col1x)/5;
    
    set1x = table2array(col3x(1:ind, "Var3"));%B1
    set2x = table2array(col3x(ind+1:2*ind, "Var3"));%XP1
    set3x = table2array(col3x(2*ind+1:3*ind, "Var3"));%C
    set4x = table2array(col3x(3*ind+1:4*ind, "Var3"));%XP2
    set5x = table2array(col3x(4*ind+1:5*ind, "Var3"));%B2
    
    %Same order as X sets
    set1y = table2array(col3y(1:ind, "Var3"));
    set2y = table2array(col3y(ind+1:2*ind, "Var3"));
    set3y = table2array(col3y(2*ind+1:3*ind, "Var3"));
    set4y = table2array(col3y(3*ind+1:4*ind, "Var3"));
    set5y = table2array(col3y(4*ind+1:5*ind, "Var3"));
    
    w_xp_xy1 = [set2x'; set2y']; 
    w_xp_xy2 = [set4x'; set4y']; 
    w_c_xy1 = [set3x'; set3y']; 
    w_b_xy1 = [set1x'; set1y']; 
    w_b_xy2 = [set5x'; set5y'];
    
    %rotate the deformation into pitch and xp
    w_xp_bxp1 = rot2*w_xp_xy1;
    w_xp_bxp2 = rot2*w_xp_xy2;
    w_b_bxp1 = rot2*w_b_xy1;
    w_b_bxp2 = rot2*w_b_xy2;
    w_c_bxp1 = rot2*w_c_xy1;
    
    
    
    % 
    
    
    % check rotation gives only change along xp
    vxp1 = rot2 * vxp1;
    vxp2 = rot2 * vxp2;
    
    % assemple mode shape deformation in array where col 1 is point 1 and col 3
    % is point 3 along pitch axis and then xp axis
    b_wxp = [w_b_bxp1(2,:)',w_c_bxp1(2,:)', w_b_bxp2(2,:)'];
    
    xp_wb = [w_xp_bxp1(1,:)',w_c_bxp1(1,:)', w_xp_bxp2(1,:)'];
    
    % calculate partial of deofrmation in xp wrt p
    deriv_b_wxp = diff(b_wxp,1,2);
    scaled_diff_p_wxp = [deriv_b_wxp(:,1)/vb1(1), deriv_b_wxp(:,2)/vb2(1)];
    
    % calculate partial of deofrmation in p wrt xp
    deriv_xp_wb = diff(xp_wb,1,2);
    scaled_diff_xp_wp = [deriv_xp_wb(:,1)/vxp1(2), deriv_xp_wb(:,2)/vxp2(2)];
    
    %dfx_dy
    dfb_dxp = mean(scaled_diff_xp_wp,2);
    %dfy_dx
    dfxp_db = mean(scaled_diff_p_wxp ,2);
    crl = [dfxp_db - dfb_dxp]
    
end

