clear;
clc;
pretty_pictures;

%% problem definition

% size of gridworld
n = 5;
nS = n^2;

% ice cream
RD = 1;
pD = [3, 3];
sD = xy_to_si(pD, n);
RS = 10;
pS = [3, 1];
sS = xy_to_si(pS, n);

% road
RW = -10;
pW = [5, 1;
      5, 2;
      5, 3;
      5, 4;
      5, 5];
sW = xy_to_si(pW, n);

% roadblocks
p_blocked = [2, 2;
             3, 2;
             2, 4;
             3, 4];
s_blocked = xy_to_si(p_blocked, n);

% initial condition
p0 = [4, 3];
s0 = xy_to_si(p0, n);

% discount factor
gamma = .5;

% time horizon
H = 100;

solver = 'policy';

%% state space

% S = {s_i | i=1,...,n^2}\{s_blocked}
s_good = setdiff(1:nS, s_blocked)';

%% action space

% A = {a_j | j=1,...,5} = {stay, up, right, down, left}

% formulate as a (dx, dy) pair
% A = {[0, 0], [0, 1], [1, 0], [0, -1], [-1, 0]};
A = [0, 0;
     0, 1;
     1, 0;
     0, -1;
    -1, 0];
nA = size(A, 1);

%% state transition probabilities

% Bernoulli distribution for action outcomes
p = 1; % probability of action having desired outcome
q = (1-p)/(nA-1); % probability of action having other outcomes

% initialize
P = zeros(nS, nA, nS); % P(s, a, s')

% build probability matrix
for i = 1:nS % loop over states
    
    % get xy coords of current state
    xyi = si_to_xy(i, n);
    
    % apply all possible actions
    xyi1 = xyi + A;
    
    % constrain to gridworld
    xyi1 = min(max(xyi1, 1), n);
    
    % convert back to state indices
    si1 = xy_to_si(xyi1, n);
    
    % remove obstacles
    % (this part relies on the assumption that s0 is not an obstacle)
    for j = 2:nA
        if ismember(si1(j), s_blocked)
            si1(j) = si1(1); % if trying to move into an obstacle, stay still
        end
    end
    
    % assign probabilities   
    for k = 1:nA % loop over actions
        
        % desired outcome
        P(i,k,si1(k)) = p;
        
        % other outcomes
        for l = 1:nA
            if l ~= k
                P(i,k,si1(l)) = P(i,k,si1(l)) + q;
            end
        end
        
    end
    
end

%% reward function

% initialize
R = zeros(size(P));

% rewards for being in ice cream store
R(sD, :, :) = RD;
R(sS, :, :) = RS;

% rewards for being in the road
for i = 1:length(sW)
    R(sW, :, :) = RW;
end

%% Q learning

% initializate Q matrix and gradient descent
Q_hat = rand(nS, nA);
dQ_norm = 1;
alpha = .1;
eps = .3;

% plot Q convergence
figure(2);
clf;
title('infinity norm of dQ');
xlabel('trajectory');
hold on;
grid on;

tau = 1;
while dQ_norm > 1e-4
    
    s_t = s_blocked(1);
    while ismember(s_t, s_blocked)
        s_t = randi(nS);
    end

%     % initialize plot
%     figure(1);
%     clf;
%     hold on;
% 
%     % ice cream
%     plot([pD(1), pS(1)], [pD(2), pS(2)], 'gs', 'LineWidth', 4, 'MarkerSize', 50);
% 
%     % road
%     plot(pW(:,1), pW(:,2), 'rs', 'LineWidth', 4, 'MarkerSize', 50);
% 
%     % buildings
%     plot(p_blocked(:,1), p_blocked(:,2), 'ks', 'LineWidth', 4, 'MarkerSize', 50);
% 
%     % boundaries
%     plot([0 n n 0 0]+1/2, [0 0 n n 0]+1/2, 'k--');
% 
%     grid on;
%     axis('equal');
%     xlim([0, n+1]);
%     ylim([0, n+1]);
%     title(sprintf('Gathering data, trial: %.0f', tau));

    Q_hat1 = Q_hat;
    for i = 1:H % loop trajectory

        % epsilon greedy policy
        if rand() < eps
            a_t = randi(nA);
        else
            [~,a_t] = max(Q_hat(s_t,:), [], 2);
        end

        % update state according to transition probabilities
        trans_prob = P(s_t, a_t, :);
        trans_rand = rand();
        s_t1 = find(trans_rand < cumsum(trans_prob), 1);
        p_t1 = si_to_xy(s_t1, n);

        % gather reward
        r_t = R(s_t, a_t, s_t1);

        % record data
%         D(H*(tau-1)+i,:) = [s_t, a_t, s_t1, r_t];
        
        % gradient descent on Q_hat1
        dQ1 = 1;
        while dQ1 > 1e-2*alpha
            dQ1 = alpha*(r_t + gamma*max(Q_hat1(s_t1, :)) - Q_hat1(s_t, a_t));
            Q_hat1(s_t,a_t) = Q_hat1(s_t,a_t) + dQ1;
        end
        
        % update data
        s_t = s_t1;
        
        % plot
%         plot(p_t1(1), p_t1(2), 'bo');

    end
%     pause(.1); % pause for plotting
    
    % check for convergence of Q_hat
    dQ = Q_hat1 - Q_hat;
    dQ_norm = norm(dQ, 'inf');
    Q_hat = Q_hat1;
    figure(2);
    plot(tau, dQ_norm, 'o');
    
    % increment trajectory
    tau = tau + 1;
    
end
figure(1); hold off;
figure(2); hold off;

%% simulate with optimal policy

% 
% s = zeros(H, 1);
% s(1) = s0;
% 
% for i = 2:H
%     
%     % get current position
%     s_t = s(i-1);
%     p_t = si_to_xy(s_t, n);
%     
%     % take action
%     a_t = A(Pi(s_t), :);
%     
%     % update state
%     p_t1 = p_t + a_t;
%     s(i) = xy_to_si(p_t1, n);
%     
% end

%% plot policy

[~, Pi] = max(Q_hat, [], 2);

figure(1);
clf;
hold on;

% starting point
% plot(p0(1), p0(2), 'k*', 'LineWidth', 2, 'MarkerSize', 25);

% ice cream
plot([pD(1), pS(1)], [pD(2), pS(2)], 'gs', 'LineWidth', 4, 'MarkerSize', 50);

% road
plot(pW(:,1), pW(:,2), 'rs', 'LineWidth', 4, 'MarkerSize', 50);

% buildings
plot(p_blocked(:,1), p_blocked(:,2), 'ks', 'LineWidth', 4, 'MarkerSize', 50);

% % path
% path = si_to_xy(s, n);
% plot(path(:, 1), path(:, 2), 'b');

% policy
xy = si_to_xy(s_good, n);
uv = A(Pi(s_good), :);
xy = xy - .15*uv;
quiver(xy(:,1), xy(:,2), uv(:,1), uv(:,2), .3, 'b');
scatter(xy(Pi(s_good) == 1, 1), xy(Pi(s_good) == 1, 2), 'bo');

% boundaries
plot([0 n n 0 0]+1/2, [0 0 n n 0]+1/2, 'k--');

hold off;
grid on;
axis('equal');
xlim([0, n+1]);
ylim([0, n+1]);
title('Learned policy');
