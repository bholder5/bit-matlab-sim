function crl = gyro1_bore()
    clear
    clc
    
    addpath('/home/brad/bit-matlab-sim/Miscellaneous/')
    % addpath('/home/brad/bit-matlab-sim/flexible_model_data/pitch_port')
    
    pp_xp.one.locx = -1045.310000;
    pp_xp.one.locy = 2585.874000;
    pp_xp.two.locx = -1050.390000;
    pp_xp.two.locy = 2592.477000;
    pp_xp.thr.locx = -1055.480000;
    pp_xp.thr.locy = 2599.080000;
    
    %vector along pitch to calculate rotation matrix
    vp3 = [pp_xp.thr.locx - pp_xp.one.locx; pp_xp.thr.locy - pp_xp.one.locy];
    vp1 = [pp_xp.two.locx - pp_xp.one.locx; pp_xp.two.locy - pp_xp.one.locy];
    vp2 = [pp_xp.thr.locx - pp_xp.two.locx; pp_xp.thr.locy - pp_xp.two.locy];
    
    dotprod = vp1' * [0;1];
    dotprod = dotprod/norm(vp1);
    ang = acos(dotprod);
    ang = rad2deg(ang);
    
    dotprod2 = vp2' * [0;1];
    dotprod2 = dotprod2/norm(vp2);
    ang2 = acos(dotprod2);
    ang2 = rad2deg(ang2);
    
    ang = mean([ang, ang2]);
    
    rot = axis2rot([0;0;1], deg2rad(ang));
    rot2 = rot(1:2,1:2);
    
    
    vp1 = rot2 * vp1;
    vp2 = rot2 * vp2;
    
    tab_x = readtable('/home/brad/bit-matlab-sim/flexible_model_data/gyro1_bore/wx.csv');
    col1x = tab_x(:,1);
    col2x = tab_x(:,2);
    col3x = tab_x(:,3);
    
    tab_y = readtable('/home/brad/bit-matlab-sim/flexible_model_data/gyro1_bore/wy.csv');
    col1y = tab_y(:,1);
    col2y = tab_y(:,2);
    col3y = tab_y(:,3);
    
    tab_z = readtable('/home/brad/bit-matlab-sim/flexible_model_data/gyro1_bore/wz.csv');
    col1z = tab_z(:,1);
    col2z = tab_z(:,2);
    col3z = tab_z(:,3);
    
    ind = height(col1x)/5;
    
    set1x = table2array(col3x(1:ind, "Var3"));%C
    set2x = table2array(col3x(ind+1:2*ind, "Var3"));%XP2
    set3x = table2array(col3x(2*ind+1:3*ind, "Var3"));%P2
    set4x = table2array(col3x(3*ind+1:4*ind, "Var3"));%P1
    set5x = table2array(col3x(4*ind+1:5*ind, "Var3"));%XP1
    
    %Same order as X sets
    set1y = table2array(col3y(1:ind, "Var3"));
    set2y = table2array(col3y(ind+1:2*ind, "Var3"));
    set3y = table2array(col3y(2*ind+1:3*ind, "Var3"));
    set4y = table2array(col3y(3*ind+1:4*ind, "Var3"));
    set5y = table2array(col3y(4*ind+1:5*ind, "Var3"));
    
    %Same order as X sets
    set1z = table2array(col3z(1:ind, "Var3"));
    set2z = table2array(col3z(ind+1:2*ind, "Var3"));
    set3z = table2array(col3z(2*ind+1:3*ind, "Var3"));
    set4z = table2array(col3z(3*ind+1:4*ind, "Var3"));
    set5z = table2array(col3z(4*ind+1:5*ind, "Var3"));
    
    w_xp1_xy = [set5x'; set5y']; 
    w_xp2_xy = [set2x'; set2y']; 
    w_c_xy1 = [set1x'; set1y']; 
    w_p1_xy = [set4x'; set4y']; 
    w_p2_xy = [set3x'; set3y'];
    
    %rotate the deformation into pitch and xp
    %w_loc_deform
    
    w_xp1_bxp = rot2*w_xp1_xy;
    w_xp2_bxp = rot2*w_xp2_xy;
    w_p1_bxp = rot2*w_p1_xy;
    w_p2_bxp = rot2*w_p2_xy;
    w_c_bxp1 = rot2*w_c_xy1;
    
    w_z_xp = [set5z, set1z, set2z] ;
    
    
    %% other axis
    
    p_wxp = [w_p1_bxp(2,:)',w_c_bxp1(2,:)', w_p2_bxp(2,:)'];
    % 
    % xp_wp = [w_p_bxp(1,:)',w_c_pxp1(1,:)', w_xp_pxp2(1,:)']
    p1 = -7.158000;
    p2 = 1.175200;
    p3 = 9.508500;
    dp = [p2-p1; p3-p2];
    
    % calculate partial of deofrmation in xp wrt p
    deriv_p_wxp = diff(p_wxp,1,2);
    scaled_diff_p_wxp = [deriv_p_wxp(:,1)/dp(1), deriv_p_wxp(:,2)/dp(2)];
    
    % calculate partial of deofrmation in p wrt xp
    deriv_xp_wp = diff(w_z_xp,1,2);
    scaled_diff_xp_wp = [deriv_xp_wp(:,1)/vp1(2), deriv_xp_wp(:,2)/vp2(2)];
    
    dwp_dxp = mean(scaled_diff_xp_wp,2)
    dwxp_dp = mean(scaled_diff_p_wxp ,2)
    
    crl = [dwp_dxp - dwxp_dp];
    
    

end