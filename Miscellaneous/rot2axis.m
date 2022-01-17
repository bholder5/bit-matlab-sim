function [v, phi] = rot2axis(C)
    phi = acos((C(1,1) + C(2,2) + C(3,3) - 1)/2);

    sinphi = sin(phi);
    v = [C(3,2) - C(2,3); C(1,3) - C(3,1); C(2,1) - C(1,2)]/(2*sinphi);
    v = v/norm(v);
end