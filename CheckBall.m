function [LeftReward, RightReward, winner, terminal] = CheckBall(P, V, LastV, ...
                                    Table, Robot1, Robot2, NetNormalForce)
%% Check if the ball is out of the boundaries (Game Cube)
% 

% With distanceTolerance=0.5 the dimensions of the cube are
% X = Table + 2x Distance Robot Table + 2x distanceTolerance
% X = 2.74 + 0.3 + 0.3 + 0.5 +0.5 = 4.34
%
% Y = Table + 2x distanceTolerance
% Y = 1.5250 + 0.5 +0.5 = 2.525
%
% Z = Height = 5 

distanceTolerance=0.5;
maxHeight=5;
winner='';
terminal=false;

LeftReward=0;
RightReward=0;

%% Touched Net
if (NetNormalForce ~=0 && LastV(1)>0)
    disp('Left Player played Net')
    disp('Right Player won')
    LeftReward=-0.7;
    RightReward=0.7;
    winner='right';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
elseif(NetNormalForce ~=0  && LastV(1)<0)
    disp('Right Player played Net')
    disp('Left Player won')
    LeftReward=0.7;
    RightReward=-0.7;
    winner='left';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end
%% Vx=0
if V(1)==0
    disp('Vx=0!')
    set_param('Pong3D','SimulationCommand','stop')
end
%% Hit Rewards
if(LastV(1)<0 && V(1) > 0)
    disp('Left Player hit the ball')
    LeftReward=0.2;
end
if(LastV(1)>0 && V(1)<0)
    disp('Right Player hit the ball')
    RightReward=0.2;
end

%% Out on Left Player side
if (P(1) < -Table.Dimensions(1)/2-Robot1.Distance_Table(1)-distanceTolerance && V(1) < 0)
    disp('Left Player did not get the ball')
    disp('Right Player won')
    LeftReward=-0.7;
    RightReward=0.7;
    winner='right';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end

%% Out on Right Player side
if (P(1) > Table.Dimensions(1)/2+Robot2.Distance_Table(1)+distanceTolerance && V(1) > 0)
    disp('Right Player did not get the ball')
    disp('Left Player won')
    LeftReward=0.7;
    RightReward=-0.7;
    winner='left';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end

%% Out on Side
if ((V(1) > 0 && P(2) > Table.Dimensions(2)/2+distanceTolerance) || ...
        (V(1) > 0 && P(2) < -Table.Dimensions(2)/2-distanceTolerance))
    disp('Left Player side out')
    disp('Right Player won')
    LeftReward=-0.7;
    RightReward=0.7;
    winner='right';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end

if ((V(1) < 0 && P(2) > Table.Dimensions(2)/2+distanceTolerance) || ...
        (V(1) < 0 && P(2) < -Table.Dimensions(2)/2-distanceTolerance))
    disp('Right Player side out')
    disp('Left Player won')
    LeftReward=0.7;
    RightReward=-0.7;
    winner='left';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end

%% Too low
if(V(1)>0 && P(3)<-Table.Height)
    disp('Left Player ball out (bottom)')
    disp('Right Player won')
    LeftReward=-0.7;
    RightReward=0.7;
    winner='right';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end
if(V(1)<0 && P(3)<-Table.Height)
    disp('Right Player ball out (bottom)')
    disp('Left Player won')
    LeftReward=0.7;
    RightReward=-0.7;
    winner='left';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end

%% Too high
if(V(1)>0 && P(3)>maxHeight)
    disp('Left Player ball out (top)')
    disp('Right Player won')
    LeftReward=-0.7;
    RightReward=0.7;
    winner='right';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end
if(V(1)<0 && P(3)>maxHeight)
    disp('Right Player ball out (top)')
    disp('Left Player won')
    LeftReward=0.7;
    RightReward=-0.7;
    winner='left';
    set_param('Pong3D','SimulationCommand','stop')
    terminal=true;
end