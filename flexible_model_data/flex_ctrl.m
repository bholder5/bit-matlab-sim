function [ac, bc, cc] = flex_ctrl(a, b, c, r, qr, ql)
    
    [x,k,l] = icare(a,b,qr,r);
    cc = k;

    ac = a - b * cc;

    eig(ac)



    P = lyap(ac', ql);

    bc = inv(P) * cc';

    %%assemble whole control statematrix
    A_comb = [a, -b*cc; bc*c ac];
    eig(A_comb)

end