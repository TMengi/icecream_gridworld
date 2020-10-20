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
RW = -1;
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
p0 = [3, 5];
s0 = xy_to_si(p0, n);

% discount factor
gamma = .3;

% time horizon
H = 10;

%% state space

% S = {s_i | i=1,...,n^2}\{s_blocked}

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
p = .9; % probability of action having desired outcome
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

% sum along third axis should be ones I think???
% fuck yeah

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

%% value iteration algorithm

% initialization
i = 0;
V = zeros(nS, 1);
Pi = randi(nA, [nS, 1]);

% loop
while i < H
    [V, Pi] = max(sum(P.*(R + gamma*ones(nS, nA, nS).*reshape(V, 1, 1, nS)), 3), [], 2);
    i = i + 1;
end

%% run simulation

s = zeros(H, 1);
s(1) = s0;

for i = 2:H
    
    % get current position
    s_t = s(i-1);
    p_t = si_to_xy(s_t, n);
    
    % take action
    a_t = A(Pi(s_t), :);
    
    % update state
    p_t1 = p_t + a_t;
    s(i) = xy_to_si(p_t1, n);
    
end

%% plot result

figure(1);

% starting point
plot(p0(1), p0(2), 'k*', 'LineWidth', 2, 'MarkerSize', 25), hold on;

% ice cream
plot([pD(1), pS(1)], [pD(2), pS(2)], 'gs', 'LineWidth', 4, 'MarkerSize', 50);

% road
plot(pW(:,1), pW(:,2), 'rs', 'LineWidth', 4, 'MarkerSize', 50);

% buildings
plot(p_blocked(:,1), p_blocked(:,2), 'ks', 'LineWidth', 4, 'MarkerSize', 50);

% path
path = si_to_xy(s, n);
plot(path(:, 1), path(:, 2), 'b');

% boundaries
plot([0 n n 0 0]+1/2, [0 0 n n 0]+1/2, 'k--');

hold off;
grid on;
axis('equal');
xlim([0, n+1]);
ylim([0, n+1]);
% title('Ice Cream Gridworld');
title(sprintf('gamma=%.1f, H=%.0f, p=%.1f', gamma, H, p));