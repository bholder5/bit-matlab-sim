function [mass_mat] = compute_mass_matrix(theta, z_n, r_n1_n, m_w_n, p_n)
%Compute_Mass_Matrix computes the mass matrix of the 9 state system using
%the angles theta to calculate the interbody transformation matrices and
%the axis of rotations.

% Initialize mass matrix
mass_mat = (zeros(9,9));

% memory to store the interbody transformations
T_n = (zeros(6,6,9));

T_ni = (zeros(6,6));
T_nj = T_ni;

J_ni = (zeros(6,1));
J_nj = J_ni;

% equation 3.8: Interbody transformations for each frame
for i = 1:9
    C_n = axis2rot(z_n(:,i), theta(i));
    off_term = xmat(r_n1_n(:,i));
    off_term = -1*C_n*off_term;
    T_n(:,:,i) = ([C_n, off_term; zeros(3,3), C_n]);
end

% Generate the mass matrix
% Eq 3.12-3.13
for i = 1:9
    for j = i:9
        M_ij = 0;
        for n = j:9
            T_ni = (eye(6));
            T_nj = T_ni;
            
            for k = i+1:j
%                 tic
                T_ni = T_n(:,:,k)*T_ni;
%                 toc
%                 tic
%                 T_ni = mtimes(T_n(:,:,k),T_ni);
%                 toc
            end
            
            for k = j+1:n
                T_nj = T_n(:,:,k) * T_nj;
%                 T_nj = mtimes(T_n(:,:,k),T_nj);
            end
            T_ni = T_nj*T_ni;
%             T_ni = mtimes(T_nj,T_ni);
            
            J_ni = (T_ni) * p_n(:,i);
            J_nj = (T_nj) * p_n(:,j);
            
            add = (J_ni' * m_w_n(:,:,n) * J_nj);
            M_ij = M_ij + add;
        end
        mass_mat(i,j) = M_ij;
        if (i ~= j)
            mass_mat(j,i) = M_ij;
        end
    end
end

end


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
