%% Play the game!

%% Preparations
clear;
bdclose all
slCharacterEncoding('ISO-8859-1')

% Load the Structs
PingPongStructs6DOF
% Load the Models from the Robotics Toolbox 
ModelRobot1Robot2

% Load the Initial Values
load('Robot1InitialValues.mat')
load('Robot2InitialValues.mat')

% Adjust the Bounds of the Robot to Get some more mobility
Robot1.Joint.Lower_Bound=[-170 -45 -66 -360 -360 -350];
Robot1.Joint.Upper_Bound=[170 190 210 360 360 350];
Robot2.Joint.Lower_Bound=[-170 -45 -66 -360 -360 -350];
Robot2.Joint.Upper_Bound=[170 190 210 360 360 350];

% Set the update frequency of the Simulink model
updateFrequency=0.01;
NetNormalForce=0;

% Open the Simulink Model
open_system('Pong3D')
% Set Acceleration Mode (No video)
% set_param('Pong3D','SimulationMode','Accelerator')
% Set FastRestart (No compiling between rounds)
%set_param('Pong3D','FastRestart','on')
pause(10)

%% Game
%% Preparation
% Get random start parameters of the Ball 
% Ball starts on right side
[P,V]=RespawnBall('right', Table);
% These are all values which the Simulink model needs
Ball.Initial_Position=P;
Ball.Initial_Velocity=V;
InitialBall=true;
LastP=P;
LastV=V;
R1Torque=[0,0,0,0,0,0];
R2Torque=[0,0,0,0,0,0];

% Counter
LeftWins=0;
RightWins=0;

% Set the Players
LeftPlayer= AIPlayer('left', Table, Robot1, Ball, Robot1Model);
RightPlayer=AIPlayer('right',Table, Robot2, Ball, Robot2Model);

% Start the Game
set_param('Pong3D','SimulationCommand','start')

% While the game is running or in paused mode
while ((strcmp(get_param('Pong3D','SimulationStatus'),('running'))) || ...
        (strcmp(get_param('Pong3D','SimulationStatus'),('paused'))))
    % Get the Data from the simulation
    [P, V, currentTime,...
        R1CurrentAngles, R1CurrentVelocity,...
        R2CurrentAngles, R2CurrentVelocity, ...
        NetNormalForce] = ...
                          GetDataFromSimulation();
    
    while(InitialBall && V(1)==0)
        pause(0.02)
        [P, V, currentTime,...
            R1CurrentAngles, R1CurrentVelocity,...
            R2CurrentAngles, R2CurrentVelocity, ...
            NetNormalForce] = ...
                              GetDataFromSimulation();
        
        disp('V(1)=0')
    end
    InitialBall=false;
    % Check if the Ball is out of the Game Cube
    [LeftReward, RightReward, winner, terminal] = ...
        CheckBall(P, V, LastV, Table, Robot1, Robot2, NetNormalForce);
    % Set Scores
    if strcmp(winner,'left')
        LeftWins=LeftWins+1;
        disp(['Left: ', num2str(LeftWins), ' Right: ', num2str(RightWins)])
    elseif strcmp(winner,'right')
        RightWins=RightWins+1;
        disp(['Left: ', num2str(LeftWins), ' Right: ', num2str(RightWins)])
    end
    % If one of the Player lost terminal==true
    if terminal
        % The Ball spawns randomly on the winner side
        [P,V]=RespawnBall(winner, Table);
        disp(['Respawn ball on ', winner, ' side'])
        Ball.Initial_Position=P;
        Ball.Initial_Velocity=V;
        InitialBall=true;
        % Restart the game
        set_param('Pong3D','SimulationCommand','update')
        set_param('Pong3D','SimulationCommand','start')

        [~, ~, currentTime,...
            R1CurrentAngles, R1CurrentVelocity,...
            R2CurrentAngles, R2CurrentVelocity, ~] = ...
                                               GetDataFromSimulation();
        % Set PredictEnd to false, since this is a new Round
        LeftPlayer.PredictEnd=false;
        RightPlayer.PredictEnd=false;

    % Everything all right --> go on
    else
        % Update left Player 
        LeftPlayer.update(P, V, ...
                          LastP, LastV, ...
                          currentTime, ...
                          R1CurrentAngles, R1CurrentVelocity);

        % Update right Player
        RightPlayer.update(P, V, ...
                           LastP, LastV , ...
                           currentTime, ...
                           R2CurrentAngles, R2CurrentVelocity)
    end
        
    % Store Last P anf V
    LastP=P;
    LastV=V;

    winner='';
    NetNormalForce=0;
    % Small pause, to let simulink process
    pause(0.02)
end