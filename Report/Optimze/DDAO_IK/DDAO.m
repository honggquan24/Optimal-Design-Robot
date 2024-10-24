%__________________________________________________________________     _%
%  Dynamic differential annealed optimization: 
%  New metaheuristic optimization algorithm for engineering applications
%  (DDAO) source codes version 1.0                                       %
%                                                                        %
%  Developed in MATLAB R2016b                                            %
%                                                                        %
%  Author and programmer: Hazim Nasir Ghafil                             %
%                                                                        %
%         e-Mail: hazimn.bedran@uokufa.edu.iq                            %
%                 hazimbedran@gmail.com                                  %
%                                                                        %
% Homepage: http://staff.uokufa.edu.iq/en/cv.php?hazimn.bedran           %                          
% Main paper:
% Hazim Nasir Ghafil, Karoly Jarmai                       
% Dynamic differential annealed optimization:New metaheuristic optimization 
% algorithm for engineering applications, Applied soft computing           
% doi: 10.1016/j.asoc.2020.106392                                        %
% https://www.sciencedirect.com/science/article/pii/S156849462030332X    %
%___________________________________________________________________     %
clc;
clear;
close all;
%% Problem Parameters
CostFunction = @(x) cost(x);    
Nvar = 5;              % Number of Variables
VarLength = [1 Nvar];   % solution vector size
L_limit= [-185,-32,-147,-5,-360];         % Decision Variables Lower Bound
U_limit= [153,149,51,180,360];         % Decision Variables Upper Bound

%% DDAO Parameters
MaxIt= 500;     % Maximum Number of Iterations
MaxSubIt=1000;    % Maximum Number of Sub-iterations
T0=2000;       % Initial Temp.
alpha=0.95;     % Temp. Reduction Rate
Npop=3;        % Population Size
%% Initialization
empty_template.Phase=[];
empty_template.Cost=[];
pop=repmat(empty_template,Npop,1);
% Initialize Best Solution
BestSol.Cost=inf;
% Initialize Population
for i=1:Npop    
    % Initialize Position
    for j=1:length(L_limit)
      pop(i).Phase(j) = unifrnd(L_limit(j),U_limit(j),1) ; 
    end
%     pop(i).Phase= unifrnd(L_limit,U_limit,VarLength);
    % Evaluation
    pop(i).Cost=CostFunction(pop(i).Phase);    
    % Update Best Solution
    if pop(i).Cost<=BestSol.Cost
        BestSol=pop(i);
    end    
end
% Vector to Hold Best Costs
BestCost=zeros(MaxIt,1);
% Intialize Temp.
T = T0;
%% main loop
for t=1:MaxIt
    newpop = repmat(empty_template,MaxSubIt,1);
    
        for subit=1:MaxSubIt        
        % Create and Evaluate New Solutions  
          for j=1:length(L_limit)
           newpop(subit).Phase(j) = unifrnd(L_limit(j),U_limit(j),1) ; 
          end
%         newpop(subit).Phase = unifrnd(L_limit,U_limit,VarLength);
        % set the new solution within the search space
        newpop(subit).Phase = max(newpop(subit).Phase, L_limit);
        newpop(subit).Phase = min(newpop(subit).Phase, U_limit);
        % Evaluate new solution
        newpop(subit).Cost=CostFunction(newpop(subit).Phase);
        end        
        % Sort Neighbors
        [~, SortOrder]=sort([newpop.Cost]);
        newpop=newpop(SortOrder);
        bnew = newpop(1);
        kk = randi(Npop);
        bb = randi(Npop);
        % forging parameter
        if(rem(t,2)==1)
        Mnew.Phase = (pop(kk).Phase-pop(bb).Phase)+ bnew.Phase;
        elseif (rem(t,2)==0)
             Mnew.Phase = (pop(kk).Phase-pop(bb).Phase)+ bnew.Phase*rand;
        end
        % set the new solution within the search space
        Mnew.Phase = max(Mnew.Phase, L_limit);
        Mnew.Phase = min(Mnew.Phase, U_limit); 
        % Evaluate new solution
        Mnew.Cost=CostFunction(Mnew.Phase);
        for i=1:Npop            
            if Mnew.Cost <= pop(i).Cost
                pop(i)= Mnew;                
            else
                DELTA=(Mnew.Cost-pop(i).Cost);
                P=exp(-DELTA/T);
                if rand <= P
                    pop(end)= Mnew;                                      
                end            
            end  
            % Update Best Solution Ever Found
            if pop(i).Cost <= BestSol.Cost
                BestSol=pop(i);
            end  
        end
    
    % Store Best Cost Ever Found
    BestCost(t)=BestSol.Cost;    
%     Display Iteration Information
    disp(['Iteration = ' num2str(t) ': Best Cost = ' num2str(BestCost(t))]);    
    % Update Temp.
    T=alpha*T;
end
y=BestSol.Cost;

% Results
figure;
%plot(BestCost,'LineWidth',2);
semilogy(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;