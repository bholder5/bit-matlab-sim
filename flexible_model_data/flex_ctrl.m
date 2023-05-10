function [ac, bc, cc] = flex_ctrl(a, b, c, r, qr, ql)
    
    [x,k,l] = icare(a,b,qr,r);
    cc = -k;

    ac = a - b * cc;

    P = lyap(ac, ql);

    bc = inv(P) * cc';
end