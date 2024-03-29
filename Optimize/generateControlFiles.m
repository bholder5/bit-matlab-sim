function generateControlFiles(param1, param2, param3, param4, param5, param6, param7, param8, param9, param10, param11, param12)

    % Initialization
    num_in = 5;
    num_modes = 0;
    num_states = 6 + 2 * num_modes;

    addpath('/home/bholder/bit-matlab-sim/')
    load('control_matrices.mat', 'A_p', 'B_p', 'C_vel', 'A_n', 'B_n');

    % Assuming A_p, B_p, C_vel, A_n, B_n, C_vel are defined or loaded previously
    % If they are not, you need to define or load them before using

    % Positive control
    A_use = A_p(1:num_states, 1:num_states);
    B_use = B_p(1:num_states, :);
    C_use = C_vel(:, 1:num_states);

        % Creating qr and ql matrices from the parameters
    qr = diag([param1, param2, param3, param4, param5, param6]);
    ql = diag([param7, param8, param9, param10, param11, param12]);

    r_lqr = 1.00 * diag([1, 1, 1, 1, 1]);
    [K, ~, ~] = lqr(A_use, B_use, qr, r_lqr, 0);
    Cc = K;
    Ac = A_use - (B_use * Cc);
    P2 = lyap(Ac', ql);
    Bc = inv(P2) * Cc';

    generate_rust_code_to_file('/home/bholder/gigabit_code/adcs/src/control/lqr/pos.rs', A_use, B_use, Ac, Bc, Cc);

    % Negative control
    A_use = A_n(1:num_states, 1:num_states);
    B_use = B_n(1:num_states, :);
    C_use = C_vel(:, 1:num_states);

    [K, ~, ~] = lqr(A_use, B_use, qr, r_lqr, 0);
    Cc = K;
    Ac = A_use - (B_use * Cc);
    P2 = lyap(Ac', ql);
    Bc = inv(P2) * Cc';

    generate_rust_code_to_file('/home/bholder/gigabit_code/adcs/src/control/lqr/neg.rs', A_use, B_use, Ac, Bc, Cc);

    % Additional operations if needed
end
