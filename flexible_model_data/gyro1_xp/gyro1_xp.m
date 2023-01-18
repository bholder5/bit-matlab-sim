function crl = gyro1_xp() 
    clear
    clc
    
    addpath('/home/brad/bit-matlab-sim/Miscellaneous/')
    % addpath('/home/brad/bit-matlab-sim/flexible_model_data/pitch_port')
    
    b.one.locx = -1043.330000;
    b.one.locy = 2692.574000;
    b.two.locx = -1036.720000;
    b.two.locy = 2697.658000;
    b.thr.locx = -1030.120000;
    b.thr.locy = 2702.742000;
    
    %vector along pitch to calculate rotation matrix
    vb3 = [b.thr.locx - b.one.locx; b.thr.locy - b.one.locy]
    vb1 = [b.two.locx - b.one.locx; b.two.locy - b.one.locy]
    vb2 = [b.thr.locx - b.two.locx; b.thr.locy - b.two.locy]
    
    dotprod = vb1' * [1;0];
    dotprod = dotprod/norm(vb1);
    ang = acos(dotprod);
    ang = rad2deg(ang)
    
    dotprod2 = vb2' * [1;0];
    dotprod2 = dotprod2/norm(vb2);
    ang2 = acos(dotprod2);
    ang2 = rad2deg(ang2)
    
    ang = mean([ang, ang2])
    
    rot = axis2rot([0;0;1], deg2rad(ang));
    rot2 = rot(1:2,1:2);
    
    
    vb1 = rot2 * vb1
    vb2 = rot2 * vb2
    
    tab_x = readtable('/home/brad/bit-matlab-sim/flexible_model_data/gyro1_xp/wx.csv');
    col1x = tab_x(:,1);
    col2x = tab_x(:,2);
    col3x = tab_x(:,3);
    
    tab_y = readtable('/home/brad/bit-matlab-sim/flexible_model_data/gyro1_xp/wy.csv');
    col1y = tab_y(:,1);
    col2y = tab_y(:,2);
    col3y = tab_y(:,3);
    
    tab_z = readtable('/home/brad/bit-matlab-sim/flexible_model_data/gyro1_xp/wz.csv');
    col1z = tab_z(:,1);
    col2z = tab_z(:,2);
    col3z = tab_z(:,3);
    
    ind = height(col1x)/5;
    
    set1x = table2array(col3x(1:ind, "Var3"));%P2
    set2x = table2array(col3x(ind+1:2*ind, "Var3"));%B1
    set3x = table2array(col3x(2*ind+1:3*ind, "Var3"));%B2
    set4x = table2array(col3x(3*ind+1:4*ind, "Var3"));%P1
    set5x = table2array(col3x(4*ind+1:5*ind, "Var3"));%C
    
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
    
    w_b1_xy = [set2x'; set2y']; 
    w_b2_xy = [set3x'; set3y']; 
    w_c_xy1 = [set5x'; set5y']; 
    w_p1_xy = [set4x'; set4y']; 
    w_p2_xy = [set1x'; set1y'];
    
    %rotate the deformation into pitch and xp
    %w_loc_deform
    
    w_xp1_bxp = rot2*w_b1_xy;
    w_xp2_bxp = rot2*w_b2_xy;
    w_p1_bxp = rot2*w_p1_xy;
    w_p2_bxp = rot2*w_p2_xy;
    w_c_bxp1 = rot2*w_c_xy1;
    
    w_z_b = [set2z, set5z, set3z] 
    
    
    %% other axis
    
    p_wb = [w_p1_bxp(1,:)',w_c_bxp1(1,:)', w_p2_bxp(1,:)']
    % 
    % xp_wp = [w_p_bxp(1,:)',w_c_pxp1(1,:)', w_xp_pxp2(1,:)']
    p1 = -6.949000;
    p2 = 1.384000;
    p3 = 9.717300;
    dp = [p2-p1; p3-p2]
    
    % calculate partial of deofrmation in xp wrt p
    deriv_p_wb = diff(p_wb,1,2);
    scaled_diff_p_wb = [deriv_p_wb(:,1)/dp(1), deriv_p_wb(:,2)/dp(2)];
    
    % calculate partial of deofrmation in p wrt xp
    deriv_b_wp = diff(w_z_b,1,2);
    scaled_diff_b_wp = [deriv_b_wp(:,1)/vb1(1), deriv_b_wp(:,2)/vb2(1)];
    
    dfp_db = mean(scaled_diff_b_wp,2);
    dfb_dp = mean(scaled_diff_p_wb ,2);
    
    crl = [dfb_dp - dfp_db];

end