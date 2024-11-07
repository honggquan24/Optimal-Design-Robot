clc;
clear;
close all;
%% Problem Definition
Nvar = 6;                       % Number of joint angles (decision variables)
VarLength = [1 Nvar];            % Solution vector size

L_limit = [-180, -70, -28, -300, -120, -300];  % Lower bounds (in degrees)
U_limit = [180, 85, 110, 300, 120, 300];       % Upper bounds (in degrees)

EEP = [1407, 0, 1855];  % Desired end-effector position

%% Test forward kinematics with initial angles
[x, y, z] = ForwardKinematic(0, 0, 0, 0, 0, 0);

%% Optimization Parameters
MaxIt = 1500;   % Max iterations
MaxSubIt = 1500; % Max sub-iterations
T0 = 2000;      % Initial temperature
alpha = 0.95;   % Cooling rate
Npop = 3;       % Population size

%% Initialization
empty_template.Phase = [];  % Store joint angles
empty_template.Cost = [];   % Store cost

% Initialize population and best solution
pop = repmat(empty_template, Npop, 1);
BestSol.Cost = inf;

% Random initialization of the population
for i = 1:Npop
    % Initialize Position
    for j=1:length(L_limit)
      pop(i).Phase(j) = unifrnd(L_limit(j),U_limit(j),1) ; 
    end
    pop(i).Cost = CostFunction(pop(i).Phase, EEP);  % Evaluate cost
    if pop(i).Cost < BestSol.Cost
        BestSol = pop(i);  % Update best solution
    end
end

BestCost = zeros(MaxIt, 1);  % Store best cost per iteration
T = T0;  % Initial temperature

%% Main Optimization Loop
for t = 1:MaxIt
    newpop = repmat(empty_template, MaxSubIt, 1);  % Sub-iteration population
    
    % Generate random solutions and ensure bounds
    for subit = 1:MaxSubIt
        for j=1:length(L_limit)
           newpop(subit).Phase(j) = unifrnd(L_limit(j),U_limit(j),1) ; % Random angles
        end  
        newpop(subit).Cost = CostFunction(newpop(subit).Phase, EEP);  % Evaluate cost
    end
    
    % Sort by cost and select best solution from sub-iterations
    [~, SortOrder] = sort([newpop.Cost]);
    bnew = newpop(SortOrder(1));
    
    % Differential update
    for i = 1:Npop
        kk = randi(Npop);
        bb = randi(Npop);
        if mod(t, 2) == 1
            Mnew.Phase = (pop(kk).Phase - pop(bb).Phase) + bnew.Phase;
        else
            Mnew.Phase = (pop(kk).Phase - pop(bb).Phase) + bnew.Phase .* rand;
        end
        
        % Ensure bounds and evaluate
        Mnew.Phase = max(min(Mnew.Phase, U_limit), L_limit);
        Mnew.Cost = CostFunction(Mnew.Phase, EEP);
        
        % Simulated Annealing Acceptance Criterion
        if Mnew.Cost < pop(i).Cost || rand <= exp(-(Mnew.Cost - pop(i).Cost) / T)
            pop(i) = Mnew;  % Replace with new solution
        end
        
        % Update best solution if improved
        if pop(i).Cost < BestSol.Cost
            BestSol = pop(i);
        end
    end
    
    BestCost(t) = BestSol.Cost;  % Store best cost
    disp(['Iteration ' num2str(t) ': Best Cost = ' num2str(BestCost(t))]);  % Display progress
    T = alpha * T;  % Temperature reduction
end

%% Display Best Solution
disp('Best Solution (Joint Angles):');
disp(BestSol.Phase);
disp('Minimum Cost (Objective Function Value):');
disp(BestSol.Cost);

% Compute end-effector position for best solution
[px_best, py_best, pz_best] = ForwardKinematic(BestSol.Phase(1), BestSol.Phase(2), BestSol.Phase(3), BestSol.Phase(4), BestSol.Phase(5), BestSol.Phase(6));
disp('End-effector position from Best Solution:');
disp(['x = ', num2str(px_best), ', y = ', num2str(py_best), ', z = ', num2str(pz_best)]);
disp('Desired End-effector position:');
disp(['xd = ', num2str(EEP(1)), ', yd = ', num2str(EEP(2)), ', zd = ', num2str(EEP(3))]);

%% Plot Best Cost Over Iterations
figure;
semilogy(BestCost, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
title('Best Cost Over Iterations');

%% Cost Function
function error = CostFunction(sol, EEP)
    [x, y, z] = ForwardKinematic(sol(1), sol(2), sol(3), sol(4), sol(5), sol(6));
    error = norm([x, y, z] - EEP);  % Euclidean distance as cost
end

%% Forward Kinematics
function [px, py, pz] = ForwardKinematic(theta1, theta2, theta3, theta4, theta5, theta6)
    % Define DH parameters and transformations
    T01 = TransformationMatrix(0, 0, 680, theta1);
    T12 = TransformationMatrix(-90, 320, 0, theta2-90);
    T23 = TransformationMatrix(0, 975, 0, theta3);
    T34 = TransformationMatrix(-90, 200, 887, theta4);
    T45 = TransformationMatrix(90, 0, 0, theta5);
    T56 = TransformationMatrix(-90, 0, 200, theta6 + 180);
    
    % Final transformation matrix
    T06 = T01 * T12 * T23 * T34 * T45 * T56;
    px = T06(1, 4); 
    py = T06(2, 4); 
    pz = T06(3, 4);  % Extract end-effector position
end

%% DH Transformation Matrix
function T = TransformationMatrix(alpha, a, d, theta)
    T = [cosd(theta), -sind(theta), 0, a;
         sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -d*sind(alpha);
         sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha), d*cosd(alpha);
         0, 0, 0, 1];
end
