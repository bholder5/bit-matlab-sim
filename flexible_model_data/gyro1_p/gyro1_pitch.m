function crl = gyro1_pitch()
    clear
    clc
    
    addpath('/home/brad/bit-matlab-sim/Miscellaneous/')
    % addpath('/home/brad/bit-matlab-sim/flexible_model_data/pitch_port')
    
    
    b.one.locx = -997.570000;
    b.one.locy = 2633.000000;
    b.two.locx = -990.967000;
    b.two.locy = 2638.084000;
    b.thr.locx = -984.364000;
    b.thr.locy = 2643.168000;
    
    xp.one.locx = -985.883000;
    xp.one.locy = 2631.481000;
    xp.two.locx = -990.967000;
    xp.two.locy = 2638.084000;
    xp.thr.locx = -996.051000;
    xp.thr.locy = 2644.687000;
    
    
    %vector along pitch to calculate rotation matrix
    vb1 = [b.two.locx - b.one.locx; b.two.locy - b.one.locy]
    vb3 = [b.thr.locx - b.one.locx; b.thr.locy - b.one.locy]
    vb2 = [b.thr.locx - b.two.locx; b.thr.locy - b.two.locy]
     
    vxp1 = [xp.two.locx - xp.one.locx; xp.two.locy - xp.one.locy]
    vxp2 = [xp.thr.locx - xp.two.locx; xp.thr.locy - xp.two.locy]
    
    dotprod = vb1' * [1;0];
    dotprod = dotprod/norm(vb1);
    ang = acos(dotprod);
    ang1 = rad2deg(ang)
    
    dotprod2 = vb2' * [1;0];
    dotprod2 = dotprod2/norm(vb2);
    ang2 = acos(dotprod2);
    ang2 = rad2deg(ang2)
    
    dotprod3 = vxp1' * [0;1];
    dotprod3 = dotprod3/norm(vxp1);
    ang3 = acos(dotprod3);
    ang3 = rad2deg(ang3)
    
    dotprod4 = vxp2' * [0;1];
    dotprod4 = dotprod4/norm(vxp2);
    ang4 = acos(dotprod4);
    ang4 = rad2deg(ang4)
    
    ang = mean([ang1, ang2, ang3, ang4])
    
    rot = axis2rot([0;0;1], deg2rad(ang));
    rot2 = rot(1:2,1:2);
    
    
    vb1 = rot2 * vb1
    vb2 = rot2 * vb2
    
    vxp1 = rot2 * vxp1
    vxp2 = rot2 * vxp2
    
    tab_x = readtable('/home/brad/bit-matlab-sim/flexible_model_data/gyro1_p/wx.csv');
    col1x = tab_x(:,1);
    col2x = tab_x(:,2);
    col3x = tab_x(:,3);
    
    tab_y = readtable('/home/brad/bit-matlab-sim/flexible_model_data/gyro1_p/wy.csv');
    col1y = tab_y(:,1);
    col2y = tab_y(:,2);
    col3y = tab_y(:,3);
    
    ind = height(col1x)/5;
    
    set1x = table2array(col3x(1:ind, "Var3"));%CENTER
    set2x = table2array(col3x(ind+1:2*ind, "Var3"));%XP1
    set3x = table2array(col3x(2*ind+1:3*ind, "Var3"));%XP2
    set4x = table2array(col3x(3*ind+1:4*ind, "Var3"));%B2
    set5x = table2array(col3x(4*ind+1:5*ind, "Var3"));%B1
    
    %Same order as X sets
    set1y = table2array(col3y(1:ind, "Var3"));
    set2y = table2array(col3y(ind+1:2*ind, "Var3"));
    set3y = table2array(col3y(2*ind+1:3*ind, "Var3"));
    set4y = table2array(col3y(3*ind+1:4*ind, "Var3"));
    set5y = table2array(col3y(4*ind+1:5*ind, "Var3"));
    
    w_xp_xy1 = [set2x'; set2y']; 
    w_xp_xy2 = [set3x'; set3y']; 
    w_c_xy1 = [set1x'; set1y']; 
    w_b_xy1 = [set5x'; set5y']; 
    w_b_xy2 = [set4x'; set4y'];
    
    %rotate the deformation into pitch and xp
    w_xp_bxp1 = rot2*w_xp_xy1;
    w_xp_bxp2 = rot2*w_xp_xy2;
    w_b_bxp1 = rot2*w_b_xy1;
    w_b_bxp2 = rot2*w_b_xy2;
    w_c_bxp1 = rot2*w_c_xy1;
    
    %% other axis
    
    % assemple mode shape deformation in array where col 1 is point 1 and col 3
    % is point 3 along pitch axis and then xp axis
    b_wxp = [w_b_bxp1(2,:)',w_c_bxp1(2,:)', w_b_bxp2(2,:)']
    
    xp_wb = [w_xp_bxp1(1,:)',w_c_bxp1(1,:)', w_xp_bxp2(1,:)']
    
    % calculate partial of deofrmation in xp wrt p
    deriv_b_wxp = diff(b_wxp,1,2);
    scaled_diff_b_wxp = [deriv_b_wxp(:,1)/vb1(1), deriv_b_wxp(:,2)/vb2(1)];
    
    % calculate partial of deofrmation in p wrt xp
    deriv_xp_wb = diff(xp_wb,1,2);
    scaled_diff_xp_wb = [deriv_xp_wb(:,1)/vxp1(2), deriv_xp_wb(:,2)/vxp2(2)];
    
    dfb_dxp = mean(scaled_diff_xp_wb,2);
    dfxp_db = mean(scaled_diff_b_wxp ,2);
    
    crl = [dfxp_db - dfb_dxp];
    
    
    

end