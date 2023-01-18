rw_z.one.loc = -6.964000;
rw_z.two.loc = 0;
rw_z.thr.loc = 6.964300;

rw_z.one.id =  1806283;
rw_z.two.id = 1806327;
rw_z.thr.id = 1807013;

tab_z = readtable('rw_z_wx.csv');
col1 = tab_z(:,1);
col2 = tab_z(:,2);
col3 = tab_z(:,3);

ind = height(col1)/3;

set1 = table2array(col3(1:ind, "Var3"));
set2 = table2array(col3(ind+1:2*ind, "Var3"));
set3 = table2array(col3(2*ind+1:3*ind, "Var3"));

data = [set1, set2, set3];
dz = [rw_z.two.loc - rw_z.one.loc, rw_z.thr.loc - rw_z.two.loc]
deriv = diff(data,1,2)

scaled_diff = [deriv(:,1)/dz(1), deriv(:,2)/dz(2)]

contrib_z = mean(scaled_diff,2)

%% other axis
rw_x.one.loc = -397.472000;
rw_x.two.loc = -390.508000;
rw_x.thr.loc = -383.543000;

rw_x.one.id =  1806281;
rw_x.two.id = 1806327;
rw_x.thr.id = 1806850;

tab_x = readtable('rw_x_wz.csv');
col1 = tab_x(:,1);
col2 = tab_x(:,2);
col3 = tab_x(:,3);

ind = height(col1)/3;

set1 = table2array(col3(1:ind, "Var3"));
set2 = table2array(col3(ind+1:2*ind, "Var3"));
set3 = table2array(col3(2*ind+1:3*ind, "Var3"));

data = [set1, set2, set3];
dx = [rw_x.two.loc - rw_x.one.loc, rw_x.thr.loc - rw_x.two.loc];
deriv = diff(data,1,2);

scaled_diff = [deriv(:,1)/dx(1), deriv(:,2)/dx(2)];

contrib_x = mean(scaled_diff,2)