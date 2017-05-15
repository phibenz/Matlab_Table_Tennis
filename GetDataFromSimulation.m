function [P,V,currentTime,R1CurrentAngles,R1CurrentVelocity,R2CurrentAngles,R2CurrentVelocity, NetNormalForce]= ... 
    GetDataFromSimulation()
% Px
Px=get_param('Pong3D/Px', 'RuntimeObject');
Px=Px.InputPort(1);
Px=Px.Data;

% Py
Py=get_param('Pong3D/Py', 'RuntimeObject');
Py=Py.InputPort(1);
Py=Py.Data;

% Pz
Pz=get_param('Pong3D/Pz', 'RuntimeObject');
Pz=Pz.InputPort(1);
Pz=Pz.Data;

P=[Px,Py,Pz];

% Vx
Vx=get_param('Pong3D/Vx', 'RuntimeObject');
Vx=Vx.InputPort(1);
Vx=Vx.Data;

% Vy
Vy=get_param('Pong3D/Vy', 'RuntimeObject');
Vy=Vy.InputPort(1);
Vy=Vy.Data;

% Vz
Vz=get_param('Pong3D/Vz', 'RuntimeObject');
Vz=Vz.InputPort(1);
Vz=Vz.Data;

V=[Vx,Vy,Vz];

% currentTime
time=get_param('Pong3D/currentTime', 'RuntimeObject');
time=time.InputPort(1);
currentTime=time.Data;

% R1CurrentAngles
R1CA=get_param('Pong3D/R1CurrentAngles', 'RuntimeObject');
R1CA=R1CA.InputPort(1);
R1CurrentAngles=R1CA.Data;

% R1CurrentVelocity
R1CV=get_param('Pong3D/R1CurrentVelocity', 'RuntimeObject');
R1CV=R1CV.InputPort(1);
R1CurrentVelocity=R1CV.Data;

% R2CurrentAngles
R2CA=get_param('Pong3D/R2CurrentAngles', 'RuntimeObject');
R2CA=R2CA.InputPort(1);
R2CurrentAngles=R2CA.Data;

% R2CurrentVelocity
R2CV=get_param('Pong3D/R2CurrentVelocity', 'RuntimeObject');
R2CV=R2CV.InputPort(1);
R2CurrentVelocity=R2CV.Data;

% NetNormalForce
NetNormalForce=get_param('Pong3D/NetNormalForce', 'RuntimeObject');
NetNormalForce=NetNormalForce.InputPort(1);
NetNormalForce=NetNormalForce.Data;
end