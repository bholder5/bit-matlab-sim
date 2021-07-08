function Pot = compute_potential_energy_term(theta, c_n, z_n, m_n, r_n1_n, g0)
    % This function is implementing the potential energy term of the
    % lagrangian. See section 3.1.1 of Romualdez Thesis, specifically
    % Eq3.25-3.26. The rate is because of the partial derivitive of V wrt
    % theta. 
    
    %Determine relative rotation matrices and rates for each joint
    ndof = length(theta);
    
    %rotation matricies
    C_n = zeros(3,3,ndof);
    C_n_rate = C_n;
    % Potential energy
    Pot = zeros(ndof,1);
    
    for n = 1:ndof
        C_n(:,:,n) = axis2rot(z_n(:,n), theta(n));
        %Partial of Rot_n with respect to theta_n
        C_n_rate(:,:,n) = -1 * xmat(z_n(:,n)) * C_n(:,:,n);
    end
    
    % Compute potential energy term. First loop is cycling through each
    % koints contribution.
    for n = 1:ndof 
        m_i = 0;
        dVdtheta_i = zeros(ndof,1);
        
        % mass of remaining link (ex. 7 + 8 + 9) is totla mass at OF joint
        for q = n:ndof
            m_i = m_i + m_n(q);
        end
        
        if (m_i ~= 0)
            t1 = zeros(ndof,1);
            t2 = t1;
            %Cycling through all joints up to joint n, the jint we are
            %currently calculating the contribution from
            %
            %terms up to n-1 links. 
            
            for j = 1:n-1
                dC_10 = eye(3);
                
                for k = 1:n-1
                    %This multiplies the rotation matricies successively
                    % until the link j where the rate is inserted instead
                    if (k == j)
                        dC_10 = C_n_rate(:,:,k) * dC_10;
                    else
                        dC_10 = C_n(:,:,k) * dC_10;
                    end
                end
                t1(j) = r_n1_n(:,n)' * dC_10 * g0;
            end
            %%% PE terms that go from 1:n
            for j = 1:n
                dC_10 = eye(3);
                
                for k = 1:n
                    %This multiplies the rotation matricies successively
                    % until the link j is reached in which case the rate
                    % is multiplied by the overall rotation
                    if (k == j)
                        dC_10 = C_n_rate(:,:,k) * dC_10;
                    else
                        dC_10 = C_n(:,:,k) * dC_10;
                    end
                end
                %dot product
                %%%% made change here c_n(n) -> c_n(:,n)
                t2 = c_n(:,n)' * dC_10 * g0;
                
                dVdtheta_i(j) = -m_i*t1(j)-t2;
            end
        end
        Pot = Pot + dVdtheta_i;
end