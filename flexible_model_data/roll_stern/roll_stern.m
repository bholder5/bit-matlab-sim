
function crl = roll_stern()

    clear
    clc
    
    % addpath('/home/brad/bit-matlab-sim/flexible_model_data/roll_stern/')
    z.one.loc = -8.50600;
    z.two.loc = 0;
    z.thr.loc = 8.5063000;
    
    z.one.id =  210594;
    z.two.id = 210670;
    z.thr.id = 211177;
    
    tab_y = readtable('/home/bholder/bit-matlab-sim/flexible_model_data/roll_stern/wy.csv');
    col1y = tab_y(:,1);
    col2y = tab_y(:,2);
    col3y = tab_y(:,3);
    
    tab_z = readtable('/home/bholder/bit-matlab-sim/flexible_model_data/roll_stern/wz.csv');
    col1z = tab_z(:,1);
    col2z = tab_z(:,2);
    col3z = tab_z(:,3);
    
    ind = height(col1y)/5;
    
    set1y = table2array(col3y(1:ind, "Var3"));%z1
    set2y = table2array(col3y(ind+1:2*ind, "Var3"));%zc/y1
    set3y = table2array(col3y(2*ind+1:3*ind, "Var3"));%z2
    set4y = table2array(col3y(3*ind+1:4*ind, "Var3"));%yc
    set5y = table2array(col3y(4*ind+1:5*ind, "Var3"));%y2
    
    %Same order as X sets
    set1z = table2array(col3z(1:ind, "Var3"));
    set2z = table2array(col3z(ind+1:2*ind, "Var3"));
    set3z = table2array(col3z(2*ind+1:3*ind, "Var3"));
    set4z = table2array(col3z(3*ind+1:4*ind, "Var3"));
    set5z = table2array(col3z(4*ind+1:5*ind, "Var3"));
    
    y1_wz = [set2z]; 
    y2_wz = [set5z]; 
    c_wz = [set4z];
    
    c_wy = [set2y];
    z1_wy = [set1y]; 
    z2_wy = [set3y];
    
    data_y_wz = [y1_wz, c_wz, y2_wz];
    data_z_wy = [z1_wy, c_wy, z2_wy];
    
    dz = [z.two.loc - z.one.loc, z.thr.loc - z.two.loc];
    deriv = diff(data_z_wy,1,2);
    
    scaled_diff = [deriv(:,1)/dz(1), deriv(:,2)/dz(2)];
    
    dfy_dz = mean(scaled_diff,2);
    
    %% other axis
    y.one.loc = 2008.958000;
    y.two.loc = 2017.034000;
    y.thr.loc = 2025.110000;
    
    dy = [y.two.loc - y.one.loc, y.thr.loc - y.two.loc];
    
    deriv = diff(data_y_wz,1,2);
    
    scaled_diff = [deriv(:,1)/dy(1), deriv(:,2)/dy(2)];
    
    dfz_dy = mean(scaled_diff,2);
    
    
    crl = [dfz_dy - dfy_dz];

end