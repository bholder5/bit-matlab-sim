
function crl = rw()
    clear
    clc
    
    
    z.one.loc = -8.12500;
    z.two.loc = 0;
    z.thr.loc = 8.12500;
    
    z.one.id =  210594;
    z.two.id = 210670;
    z.thr.id = 211177;
    
    tab_x = readtable('/home/brad/bit-matlab-sim/flexible_model_data/RW/wx.csv');
    col1x = tab_x(:,1);
    col2x = tab_x(:,2);
    col3x = tab_x(:,3);
    
    tab_z = readtable('/home/brad/bit-matlab-sim/flexible_model_data/RW/wz.csv');
    col1z = tab_z(:,1);
    col2z = tab_z(:,2);
    col3z = tab_z(:,3);
    
    ind = height(col1x)/5;
    
    set1y = table2array(col3x(1:ind, "Var3"));%z1
    set2y = table2array(col3x(ind+1:2*ind, "Var3"));%c
    set3y = table2array(col3x(2*ind+1:3*ind, "Var3"));%z2
    set4y = table2array(col3x(3*ind+1:4*ind, "Var3"));%x1
    set5y = table2array(col3x(4*ind+1:5*ind, "Var3"));%x2
    
    %Same order as X sets
    set1z = table2array(col3z(1:ind, "Var3"));
    set2z = table2array(col3z(ind+1:2*ind, "Var3"));
    set3z = table2array(col3z(2*ind+1:3*ind, "Var3"));
    set4z = table2array(col3z(3*ind+1:4*ind, "Var3"));
    set5z = table2array(col3z(4*ind+1:5*ind, "Var3"));
    
    x1_wz = [set4z]; 
    x2_wz = [set5z]; 
    c_wz = [set2z];
    
    c_wx = [set2y];
    z1_wx = [set1y]; 
    z2_wx = [set3y];
    
    data_y_wz = [x1_wz, c_wz, x2_wz];
    data_z_wy = [z1_wx, c_wx, z2_wx];
    
    dz = [z.two.loc - z.one.loc, z.thr.loc - z.two.loc];
    deriv = diff(data_z_wy,1,2);
    
    scaled_diff = [deriv(:,1)/dz(1), deriv(:,2)/dz(2)];
    
    %dFx/dZ
    dfx_dz = mean(scaled_diff,2);
    
    %% other axis
    x.one.loc = -398.633000;
    x.two.loc = -390.508000;
    x.thr.loc =-382.383000;
    
    dx = [x.two.loc - x.one.loc, x.thr.loc - x.two.loc];
    
    deriv = diff(data_y_wz,1,2);
    
    scaled_diff = [deriv(:,1)/dx(1), deriv(:,2)/dx(2)];
    
    %dFz/dX
    dfz_dx = mean(scaled_diff,2);
    
    crl = [dfx_dz - dfz_dx];
end
