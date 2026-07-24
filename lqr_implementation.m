%% Phase 5: Linearization & LQR Balancing Design (FROM SCRATCH)
D0 = M_t*(J + m*l^2) - (m*l)^2;

% State Space Matrices around upright equilibrium [x; x_dot; theta; theta_dot] = [0; 0; 0; 0]
A = [0, 1, 0, 0;
     0, -b*(J + m*l^2)/D0,  (m^2*g*l^2)/D0,  -C*m*l/D0;
     0, 0, 0, 1;
     0, -b*m*l/D0,          M_t*m*g*l/D0,    -C*M_t/D0];

B = [0; (J + m*l^2)/D0; 0; m*l/D0];

% LQR Weighting Matrices
Q = diag([10, 1, 100, 1]); 
R = 0.01; 

disp('Toolbox LQR computed gain matrix K:');
disp(K);
%% =========================================================================
%% MOST UNDERSTANDABLE LQR SOLVER: Iterative Differential Riccati Equation
%% =========================================================================
% Continuous Euler Integration of CARE

% 1. Initialization
P = zeros(size(A)); % At terminal time t = T, remaining cost-to-go is zero
dt = 0.0001;        % Small integration step size
tolerance = 1e-7;   % Convergence stopping condition
max_iters = 100000; % Safety limit

diff = Inf;
iter = 0;

% Precompute constant inverse for speed
R_inv = inv(R);

% 2. Integrate backward in time until P stops changing
while (diff > tolerance) && (iter < max_iters)
    
    % Compute derivative dP/dt from current P
    % dP = A'*P + P*A - P*B*inv(R)*B'*P + Q
    dP = A'*P + P*A - (P * B * R_inv * B' * P) + Q;
    
    % Take a small step forward in P (which moves backward in time)
    P_next = P + dP * dt;
    
    % Check how much P changed in this iteration
    diff = norm(P_next - P, 'fro'); 
    
    % Update P for next iteration
    P = P_next;
    iter = iter + 1;
end

% 3. Calculate Gain Matrix K from final steady-state P
K_custom = R \ (B' * P); % Equivalent to inv(R) * B' * P

disp(['LQR converged in ', num2str(iter), ' iterations.']);
disp('Custom Gain K:');
disp(K_custom);

