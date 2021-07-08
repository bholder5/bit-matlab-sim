function rot = axis2rot( v, phi)
% This function gives the rotation matric applied to other rotation
% matricies, not the vector (it is transpose of the rot mat applied to the
% vector.
    cosa = cos(phi);
    sina = sin(phi);
    
    sign = 1;
    rot = (zeros(3,3));
    
    for k = 1:3
        for j = k:3
            mij = (1-cosa)*v(k)*v(j);
            if (k == j)
                mij = mij + cosa;
                rot(k,j) = mij;
            else
                %index is 3 - j - k for 0 indexed programming languages
                rot(k,j) = mij + (sign*sina*v((5-k-j)+1));
                rot(j,k) = mij - (sign*sina*v((5-k-j)+1));
                sign = sign*-1;
            end
        end
    end
end