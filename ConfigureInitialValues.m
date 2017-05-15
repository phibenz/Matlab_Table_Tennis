function [ANGLE0, DIRECTION0]=ConfigureInitialValues(robot, robotModel)
%% Description
% This function has the aim to set Initial values to solve the Inverse
% Kinematics of the Robots. 
% ANGLE0 stores the Angles 1 to 5. DIRECTION0 stores the Coordinates of the
% Position of the tool (x,y,z) and the Orientation (Roll, Pitch, Yaw). 
% This file takes some time and has only to be executed, after changes
% at the dimensions of the robot. 

%% Joint Bounds
Joint1Lower=-120;
Joint1Upper=120;

Joint2Lower=-45;
Joint2Upper=100;

% Joint3Lower=-210;
% Joint3Upper=66;
% 
% Joint4Upper
% Joint4Lower
% 
% Joint5Upper
% Joint5Lower
% 
Joint6Lower=-185;
Joint6Upper=185;
%% Set Angle Steps
% Here the step size in which the different Values are taken are defined.
alpha1=Joint1Lower:30:Joint1Upper;
max_alpha1=numel(alpha1);

alpha2=Joint2Lower:29:Joint2Upper;
max_alpha2=numel(alpha2);

alpha3=robot.Joint.Lower_Bound(3):34.5:robot.Joint.Upper_Bound(3);
max_alpha3=numel(alpha3);

alpha4=robot.Joint.Lower_Bound(4):46.25:robot.Joint.Upper_Bound(4);
max_alpha4=numel(alpha4);

alpha5=robot.Joint.Lower_Bound(5):40:robot.Joint.Upper_Bound(5);
max_alpha5=numel(alpha5);

alpha6=Joint6Lower:46.25:Joint6Upper;
max_alpha6=numel(alpha6);

%% Allocate Space
limit=max_alpha1*max_alpha2*max_alpha3*max_alpha4*max_alpha5*max_alpha6;
ANGLE0=zeros(limit,6);
DIRECTION0=zeros(limit,6);

%% Calculation
for i=1:max_alpha1
    disp([num2str(i),'/', num2str(max_alpha1)])
    for j=1:max_alpha2
        for k=1:max_alpha3
            for l=1:max_alpha4
                for m=1:max_alpha5;
                    for n=1:max_alpha6
                        pos=(i-1)*max_alpha2*max_alpha3*max_alpha4*max_alpha5*max_alpha6+(j-1)*max_alpha3*max_alpha4*max_alpha5*max_alpha6+(k-1)*max_alpha4*max_alpha5*max_alpha6+(l-1)*max_alpha5*max_alpha6+(m-1)*max_alpha6+n;
                        [Position, Orientation]=ForwardKinematics(robotModel, alpha1(i), alpha2(j), alpha3(k), alpha4(l), alpha5(m), alpha6(n));
                        ANGLE0(pos,:)=[alpha1(i), alpha2(j), alpha3(k), alpha4(l), alpha5(m), alpha6(n)];
                        DIRECTION0(pos,:)=[Position(1), Position(2), Position(3), Orientation(1), Orientation(2), Orientation(3)];                
                    end
                end
            end
        end
    end
end

%% Save Variables
if(robot.Number==1)
    Robot1Angle0=ANGLE0;
    Robot1Direction0=DIRECTION0;
    save('Robot1InitialValues','Robot1Angle0','Robot1Direction0')
elseif(robot.Number==2)
    Robot2Angle0=ANGLE0;
    Robot2Direction0=DIRECTION0;
    save('Robot2InitialValues','Robot2Angle0','Robot2Direction0')
end

end