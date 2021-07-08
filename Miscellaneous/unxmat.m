function vec = unxmat(C)
    vec = zeros(3,1);
    
    vec(1) = (C(3,2) - C(2,3))/2;
    vec(2) = (C(1,3) - C(3,1))/2;
    vec(3) = (C(2,1) - C(1,2))/2;
end