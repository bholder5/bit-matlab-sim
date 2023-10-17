function [x_opt, eigf, eigc] = opt_ql(A_use, B_use, C_use, qr, r)
    % Initial guess for the parameters (can be changed)
    x0 = ones(91,1);
    % x0(1:9) = 100000*x0(1:9);

    % Bounds for the parameters
    % lb = 0.001*ones(13,1);
    % ub = 100000*ones(13,1);

    n = 6;
    len_v = n*(n+1)/2;

    % A small positive number
    epsilon = 1e2;

    % Upper and lower bounds
    lb = -epsilon * ones(n, 1);
    ub =  epsilon * ones(n, 1);

    % Positive diagonal entries
    idx = 1;
    for i = 1:n
        lb(idx, idx) = epsilon;
        idx = idx + i;
    end


    % lb(14:18) = 0.01*ones(5,1);
    % ub(14:18) = 1000*ones(5,1);

    % Using fmincon for optimization
    options = optimoptions('particleswarm','Display','iter','MaxStallIterations',10, 'SwarmSize',50000, 'UseParallel', true);
    [x_opt, fval] = particleswarm(@(x) costFunction(x, A_use, B_use, C_use, qr, r),6, lb, ub, options);

    disp('Optimized Parameters:');
    disp(x_opt);
    disp('Cost function value:');
    disp(fval);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    a_ss = A_use;
    b_ss = B_use;
    c_ss = C_use;

    % x1 = x_opt(1:9);
    % x2 = x_opt(10:18);
    % qr = diag(x1);

    % Number of matrix rows/columns
    n = 13;

    % Initialize the matrix
    M = zeros(n, n);
    v = x_opt;

    % Fill the lower triangular part of M
    idx = 1;
    for i = 1:n
        for j = 1:i
            M(i, j) = v(idx);
            idx = idx + 1;
        end
    end

    % Fill the upper triangular part of M using symmetry
    for i = 1:n
        for j = (i+1):n
            M(i, j) = M(j, i);
        end
    end

    % Convert M to positive definite using Cholesky factorization
    % L = chol(M, 'lower');
    ql = M * M';


    % qr = diag([0.001, ...
    %         0.0001, ...
    %         0.0001,...
    %         454, ...
    %         454, ...
    %         200, ...
    %         0.0001, ...
    %         0.0001, ...
    %         0.0001, ...
    %         100000, ...
    %         100000, ...
    %         50000, ...
    %         0.0001]);

    % r = 1.0* diag([0.1, 0.05, 0.1, 1000, 1000]);
    % r = diag(x_opt(14:18));
    [K,~,~] = lqr(a_ss, b_ss, qr, r, 0);
    
    Cc = K;
    Ac = a_ss - (b_ss * Cc);
    P2 = lyap(Ac', ql);
    Bc = inv(P2) * Cc';
    A_pol = [a_ss, -b_ss * Cc; Bc * c_ss, Ac];
    eigf = eig(A_pol);
    eigc = eig(Ac);


end

function f = costFunction(x,a_ss, b_ss, c_ss, qr, r)
    % Definitions for the function that relates the parameters to the real values.
    % x1 = x(1:9);
    % x2 = x(10:18);
    
    
% Number of matrix rows/columns
    % n = 13;
    % 
    % % Initialize the matrix
    % M = zeros(n, n);
    % v = x;

    % % Fill the lower triangular part of M
    % idx = 1;
    % for i = 1:n
    %     for j = 1:i
    %         M(i, j) = v(idx);
    %         idx = idx + 1;
    %     end
    % end
    % 
    % % Fill the upper triangular part of M using symmetry
    % for i = 1:n
    %     for j = (i+1):n
    %         M(i, j) = M(j, i);
    %     end
    % end
    % 
    % % Convert M to positive definite using Cholesky factorization
    % L = chol(M, 'lower');
    % ql = M * M';
    ql = diag(x);

    % qr = diag([0.001, ...
    %         0.0001, ...
    %         0.0001,...
    %         454, ...
    %         454, ...
    %         200, ...
    %         0.0001, ...
    %         0.0001, ...
    %         0.0001, ...
    %         100000, ...
    %         100000, ...
    %         50000, ...
    %         0.0001]);

    % r = 1.0* diag([0.1, 0.05, 0.1, 1000, 1000]);
    % r = diag(x(14:18));
    [K,~,~] = lqr(a_ss, b_ss, qr, r, 0);
    
    Cc = K;
    Ac = a_ss - (b_ss * Cc);
    P2 = lyap(Ac', ql);
    Bc = inv(P2) * Cc';
    A_pol = [a_ss, -b_ss * Cc; Bc * c_ss, Ac];
    eigf = eig(A_pol);
    

    % realValues = real(eigf);
    % 
    % % Separate values into positive and negative
    % positiveValues = realValues(realValues >= 0);
    % negativeValues = realValues(realValues < 0);

    % Extract the most positive and most negative values
    % maxPositive = max(positiveValues);
    % maxNegativeL = min(negativeValues);

    eigc = eig(Ac);
    % realValues = real(eigc);
    % 
    % % Separate values into positive and negative
    % negativeValues = realValues(realValues < 0);
    % 
    % % Extract the most positive and most negative values
    % maxNegativeR = min(negativeValues);

    % Construct the cost function
    % f = -real(maxNegativeR - maxNegativeL) + (2.5*norm(real(eigc)) - norm(real(eigf))) + real(1000*maxPositive);
   
    % tol = 1e-4;  % A small tolerance to identify close eigenvalues
    % 
    % % Separating real parts of the eigenvalues
    % realEigc = real(eigc);
    % realEigf = real(eigf);
    % 
    % % Identify eigf values close to eigc
    % isCloseToEigc = false(size(realEigf));
    % for i = 1:length(realEigc)
    %     isCloseToEigc = isCloseToEigc | (abs(realEigf - realEigc(i)) < tol);
    % end
    % 
    % % Remaining values of eigf
    % remainingEigf = realEigf(~isCloseToEigc);
    % 
    % % Calculate the minimum real part from the remaining eigenvalues in eigf
    % minRemainingEigf = min(remainingEigf);
    % 
    % % Penalty terms
    % penaltyEigc = 100*sum(max(0, realEigc));   % Penalize any positive real part in eigc
    % penaltyEigf = 100*sum(max(0, realEigf));   % Penalize any positive real part in eigf
    % 
    % % Construct the cost function
    % % Maximize the difference between realEigc and minRemainingEigf (by minimizing the negative of that)
    % % and add the penalty terms
    % f = -mean(realEigc - minRemainingEigf) + penaltyEigc + penaltyEigf;

    % ... (rest of your code that calculates eigc and eigf)

    tol = 1e-4;  % A small tolerance to identify close eigenvalues

    % Separating real parts of the eigenvalues
    realEigc = sort(real(eigc), 'ComparisonMethod', 'real');
    realEigf = sort(real(eigf), 'ComparisonMethod', 'real');

    % Identify eigf values close to eigc
    isCloseToEigc = false(size(realEigf));
    for i = 1:length(realEigc)
        isCloseToEigc = isCloseToEigc | (abs(realEigf - realEigc(i)) < tol);
    end

    % Remaining values of eigf
    remainingEigf = realEigf(~isCloseToEigc);

    % Calculate the minimum real part from the remaining eigenvalues in eigf
    minRemainingEigf = min(remainingEigf);

    % Penalty terms
    % penaltyEigc = 100000*sum(max(0, realEigc));   % Penalize any positive real part in eigc
    penaltyEigf = 10000*sum(max(0, realEigf));   % Penalize any positive real part in eigf

    % Construct the penalty for the first eigenvalue being more than 10 times larger 
    % than the next three eigenvalues (in terms of real parts)
    eigenPenalty = 0;
    e1 = realEigc(1);
    e2 = realEigc(2);
    e3 = realEigc(3);
    e4 = realEigc(4);

    if e1 < 1.25*e2
        eigenPenalty = eigenPenalty + 5*(e2-e1);
    end
    if e1 < 1.25*e3
        eigenPenalty = eigenPenalty + 5*(e3-e1);
    end
    if e1 < 1.25*e4
        eigenPenalty = eigenPenalty + 5*(e4-e1);
    end

    max_eig = -0.5;

    eigenPenalty = eigenPenalty + 150*(max_eig - e1);

    % Construct the cost function
    % Maximize the difference between realEigc and minRemainingEigf (by minimizing the negative of that)
    % and add the penalty terms


    f = penaltyEigf + eigenPenalty;



    % Ensure the eigenvalues are sorted by their real parts in ascending order
    eigc = sort(eigc, 'ascend', 'ComparisonMethod', 'real');
    eigf = sort(eigf, 'ascend', 'ComparisonMethod', 'real');

    a = 1;
    b = 1;
    c = 5.25;
    d = 0.0;
    lambda_target = 5.0;

    % LQR Eigenvalue Cost
    J1 = 0;
    % for i = 1:length(eigc)
    %     J1 = J1 + a*(real(eigc(i)) + lambda_target)^2 + b*imag(eigc(i))^2;
    % end

    % Lyapunov Eigenvalue Cost
    J2 = 0;
    for j = 1:length(eigf)
        mult = 0;
        if sign(real(eigf(j))) == 1
            mult = 1;
            J2 = J2 + 10;
        end

        J2 = J2 + mult*c*real(eigf(j))^2 + d*imag(eigf(j))^2;
    end

    % Total Cost
    f = J1 + J2;
end

