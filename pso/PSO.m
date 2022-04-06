function [theta1, theta2] = PSO(xc1,yc1,xg1,yg1,xc2,yc2,xg2,yg2,v0,r)
%% Problem Definition
nVar = 2;             % Number of Decision Variables
VarSize = [1 nVar];   % Size of Decision Variables Matrix
VarMin =    0;         % Lower Bound of Variables
VarMax = 2*pi;         % Upper Bound of Variables

%% PSO Parameters
MaxIt=100;      % Maximum Number of Iterations
nPop=100;        % Population Size (Swarm Size)
w=1;             % Inertia Weight
wdamp=0.99;      % Inertia Weight Damping Ratio
c1=1.5;          % Personal Learning Coefficient
c2=2.0;          % Global Learning Coefficient

% Velocity Limits
VelMax=0.1*(VarMax-VarMin);
VelMin=-VelMax;

%% Initialization
empty_particle.Position=[];
empty_particle.Cost=[];
empty_particle.Velocity=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];

particle=repmat(empty_particle,nPop,1);
GlobalBest.Cost=inf;

for i=1:nPop
    
    % Initialize Position
    particle(i).Position=unifrnd(VarMin,VarMax,VarSize);
    
    % Initialize Velocity
    particle(i).Velocity=zeros(VarSize);
    
    % Evaluation
    theta1 = particle(i).Position(1,1);
    theta2 = particle(i).Position(1,2);
    xn1 = xc1 + v0*cos(theta1);
    yn1 = yc1 + v0*sin(theta1);
    xn2 = xc2 + v0*cos(theta2);
    yn2 = yc2 + v0*sin(theta2);
    F1 = OF1(xc1,yc1,xg1,yg1,xn1,yn1,xc2,yc2,xg2,yg2,xn2,yn2);
    F2 = OF2(xn1,yn1,xn2,yn2,r);
    F3 = OF3(xc1,yc1,xn1,yn1,xg1,yg1,xc2,yc2,xn2,yn2,xg2,yg2);
    particle(i).Cost = ObjectiveFunction(F1,F2,F3);
    
    % Update Personal Best
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost
        
        GlobalBest=particle(i).Best;
        
    end
    
end

%% PSO Main Loop
for it=1:MaxIt
    
    for i=1:nPop
        
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        
        % Apply Velocity Limits
        particle(i).Velocity = max(particle(i).Velocity,VelMin);
        particle(i).Velocity = min(particle(i).Velocity,VelMax);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Velocity Mirror Effect
        IsOutside=(particle(i).Position<VarMin | particle(i).Position>VarMax);
        particle(i).Velocity(IsOutside)=-particle(i).Velocity(IsOutside);
        
        % Apply Position Limits
        particle(i).Position = max(particle(i).Position,VarMin);
        particle(i).Position = min(particle(i).Position,VarMax);
        
        % Evaluation
        theta1 = particle(i).Position(1,1);
        theta2 = particle(i).Position(1,2);
        xn1 = xc1 + v0*cos(theta1);
        yn1 = yc1 + v0*sin(theta1);
        xn2 = xc2 + v0*cos(theta2);
        yn2 = yc2 + v0*sin(theta2);
        F1 = OF1(xc1,yc1,xg1,yg1,xn1,yn1,xc2,yc2,xg2,yg2,xn2,yn2);
        F2 = OF2(xn1,yn1,xn2,yn2,r);
        F3 = OF3(xc1,yc1,xn1,yn1,xg1,yg1,xc2,yc2,xn2,yn2,xg2,yg2);
        particle(i).Cost = ObjectiveFunction(F1,F2,F3);
        
        % Update Personal Best
        if particle(i).Cost<particle(i).Best.Cost
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost<GlobalBest.Cost
                
                GlobalBest=particle(i).Best;
                
            end
            
        end
        
    end
    
    w=w*wdamp;
    
end

theta1 = GlobalBest.Position(1,1);
theta2 = GlobalBest.Position(1,2);

end