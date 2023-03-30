function [Bc] = spr(A_red, B_red)
    %UNTITLED2 Summary of this function goes here
    %   Detailed explanation goes here
    A_red = A_mf(1:(2*num_freqs), 1:(2*num_freqs));
    B_red = B_mf(1:(2*num_freqs), :);
    
    q = eye(size(A_red));
    r = eye(length(B_red(1,:)));
    
    [K,P1,Pol] = lqr(A_red,B_red,q,r)
    
    Cc = -K;
    
    %% make closed loop A
    
    Ac = A_red - (B_red * Cc)
    
    %% solve lyapupnov equation
    q2 = eye(size(q));
    
    P2 = lyap(Ac,q2);
    
    Bc = inv(P2)*Cc';

    [x,k,l] = icare(A_red,B_red,q,r);
end