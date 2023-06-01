function [freqs] = extract_freqs()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    tab_x = readtable('/home/bholder/bit-matlab-sim/flexible_model_data/gyro2_p/wx.csv');
    col1x = tab_x(:,1);
    col2x = tab_x(:,2);
    col3x = tab_x(:,3);

    freqs = [];
    flag = 1;
    
    for k = 1:length(col2x.Var2)
        for k2 = 1:length(freqs)
            if col2x.Var2(k) == freqs(k2)
                flag = 0;
            end
        end
    
        if flag
            freqs(length(freqs)+1) = col2x.Var2(k);
        end
    
        flag = 1;
    end
    
    freqs';
end