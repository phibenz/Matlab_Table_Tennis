classdef AIPlayer<handle
%% Simple Artificial Player
% This player knows a lot of data of the ball and calculates its own
% trajectorys to play the ball back
    
    properties (SetAccess=private)
        position            % left, right
        Table
        Angle0
        Direction0
        RobotModel
        q
        qd
        qdd
        TimeVec
        g
    end
    properties
        Robot
        Ball
        PredictEnd
    end
    methods
        function AIPl=AIPlayer(position, Table, Robot, Ball, RobotModel)
        %% Configure and set up the player
            if (strcmp(position,'right') || strcmp(position, 'left'))
                AIPl.position=position;
            else
                error('Invalid position for player. (''right''|''left'')')
            end
            AIPl.Table=Table;
            AIPl.Robot=Robot;
            AIPl.Ball=Ball;
            AIPl.RobotModel=RobotModel;
            AIPl.PredictEnd=false;
            AIPl.g=9.81;
            
            if Robot.Number==1
                load('Robot1InitialValues.mat')
                AIPl.Angle0=Robot1Angle0;
                AIPl.Direction0=Robot1Direction0;
            elseif Robot.Number==2
                load('Robot2InitialValues.mat')
                AIPl.Angle0=Robot2Angle0;
                AIPl.Direction0=Robot2Direction0;
            else
                error('Invalid Robot Number. (1|2)')
            end
        end 
        function [P,V,LastP,LastV]=AdjustPositionAndVelocity(AIPl,P,V,LastP,LastV)
        %% Set Position and Velocity
        % Adjust the Position and the Velocity value according to the 
        % position of the payer
            if strcmp(AIPl.position,'right')
                P(1)=-P(1);
                P(2)=-P(2);
                LastP(1)=-LastP(1);
                LastP(2)=-LastP(2);
                V(1)=-V(1);
                V(2)=-V(2);
                LastV(1)=-LastV(1);
                LastV(2)=-LastV(2);
            else
            end
        end
        function [endDirection, endTime,  PaddleVec]...
                      = Calc_Ball_Trajectory(AIPl, P, V, startTime)
        %% Calculate which configuration the Robot shall have to Hit the Ball
        % This function comes from the former version of the model (PingPong6DOF)
        % 
        % Calculate the Hit Position
        % CalculateIncoming Velocity Vector
        % Calculate Target Position (Outgoing Vector)
        % Calculate the Time and Velocity to Return the Ball to the Target Position
        % Calculate the desired Velocity of the Paddle 
        % Transform 3D collision to 2D collision
        % Adjust the Direction of the paddle
        % Calculate Vector of Paddle

        %% Calculate the Hit Position
        
        % Time until Ball hits the table
        a=-AIPl.g/2;
        b=V(3);
        c= P(3)-(AIPl.Table.Height+AIPl.Ball.Radius);
        t11=(-b+sqrt(b^2-4*a*c))/(2*a);
        t12=(-b-sqrt(b^2-4*a*c))/(2*a);
        hitTable=false;
        zVel=V(3);
        if imag(t11)==0 && imag(t12)==0 && t11>=0 && t12>=0
            x11=V(1)*t11+P(1);
            y11=V(2)*t11+P(2);
            x12=V(1)*t12+P(1);
            y12=V(2)*t12+P(2);
            if x11>=-AIPl.Table.Dimensions(1)/2 && ...
               x11<= AIPl.Table.Dimensions(1)/2 && ...
               y11>=-AIPl.Table.Dimensions(2)/2 && ...
               y11<= AIPl.Table.Dimensions(2)/2
                hitTable=true;
                tableHitPos=[x11, y11, -AIPl.g/2*t11^2+V(3)*t11+P(3)];
                zVel=-(-AIPl.g*t11+V(3));
                tHitTable=t11;
            elseif x12>=-AIPl.Table.Dimensions(1)/2 && ...
               x12<= AIPl.Table.Dimensions(1)/2 && ...
               y12>=-AIPl.Table.Dimensions(2)/2 && ...
               y12<= AIPl.Table.Dimensions(2)/2
                hitTable=true;
                tableHitPos=[x12, y12, -AIPl.g/2*t12^2+V(3)*t12+P(3)];
                zVel=-(-AIPl.g*t12+V(3));
                tHitTable=t12;
            end
        elseif imag(t11)==0 && t11>=0
            x11=V(1)*t11+P(1);
            y11=V(2)*t11+P(2);
            if x11>=-AIPl.Table.Dimensions(1)/2 && ...
               x11<= AIPl.Table.Dimensions(1)/2 && ...
               y11>=-AIPl.Table.Dimensions(2)/2 && ...
               y11<= AIPl.Table.Dimensions(2)/2
                hitTable=true;
                tableHitPos=[x11, y11, -AIPl.g/2*t11^2+V(3)*t11+P(3)];
                zVel=-(-AIPl.g*t11+V(3));
                tHitTable=t11;
            end
        elseif imag(t12)==0 && t12>=0
            x12=V(1)*t12+P(1);
            y12=V(2)*t12+P(2);
            if x12<= AIPl.Table.Dimensions(1)/2 && ...
               y12>=-AIPl.Table.Dimensions(2)/2 && ...
               y12<= AIPl.Table.Dimensions(2)/2
                hitTable=true;
                tableHitPos=[x12, y12, -AIPl.g/2*t12^2+V(3)*t12+P(3)];
                zVel=-(-AIPl.g*t12+V(3));
                tHitTable=t12;
            end
        else
            warning('Could not find a time t11 or t12')
        end
        
        cooDist=-AIPl.Table.Dimensions(1)/2-AIPl.Robot.Distance_Table(1);
        % Moment when ball is at height of Robot1:
        txRo=(cooDist-P(1))/V(1);
        % Y-Position for this moment txRo
        yRo=P(2)+V(2)*txRo;
        % Slope of the x,y graph
        m=V(2)/V(1);
        %Formula mx'+yRo=y
        %    -0.465y^2+0.42=x'
        %    -0.465y^2 + 0.42 = (y-yRo)/m
        %--> -0.465*m*y^2 -y +0.42*m+yRo
        a=-0.465*m;
        b=-1;
        c=+0.42*m+yRo;
        y11=(-b+sqrt(b^2-4*a*c))/(2*a);
        y12=(-b-sqrt(b^2-4*a*c))/(2*a);
        
        if imag(y11)~=0 && imag(y12)~=0
            disp('No intersection between the two graphs')
            hitPosition=[0.6025 0 0.89];
            roll=0;
            pitch=0;
            yaw=0;
            addVel=[0,0,0];
            tHitPos=(hitPosition(1,2)-P(2))/V(2);
        else
            hitPosition=zeros(1,3);
            hitPosition(1,2)=y12;
            hitPosition(1,1)=(y12-yRo)/m;
            tHitPos=(hitPosition(1,2)-P(2))/V(2);
            if hitTable
                tDiff=tHitPos-tHitTable;
                hitPosition(1,3)=tableHitPos(1,3)+zVel*tDiff-AIPl.g/2*tDiff^2;
                hitZVel=zVel-AIPl.g*tDiff;
            else
                hitPosition(1,3)=P(3)+V(3)*tHitPos-AIPl.g/2*tHitPos^2;
                hitZVel=V(3)-AIPl.g*tHitPos^2;
            end
        
    %% Target Position (Outgoing Vector) 
            % Random numbers for x and y Position on the Table
            xTargetPos=AIPl.Table.Dimensions(1)/4;
            %xTargetPos=0.9;
            yTargetPos=-AIPl.Table.Dimensions(2)/4+rand(1)*(AIPl.Table.Dimensions(2)/2);
            %yTargetPos=-0.30+rand(1)*0.6;
            zTargetPos=AIPl.Table.Height+AIPl.Ball.Radius;
            TargetPos=[xTargetPos, yTargetPos, zTargetPos];

            tReturn=0.6;
            xVReturn=(xTargetPos-(hitPosition(1,1)+cooDist))/tReturn;
            yVReturn=(yTargetPos-hitPosition(1,2))/tReturn;
            zVReturn=(zTargetPos-hitPosition(1,3)+AIPl.g/2*tReturn^2)/tReturn;

            vIn=[V(1), V(2), hitZVel];
            vOut=[xVReturn, yVReturn, zVReturn];
            
            ratio=norm(vOut)/norm(vIn);
            count=0;
            while(ratio >1.1 || ratio<0.9) && count<15
                if ratio <0.9
                    xTargetPos=xTargetPos+0.04;
                    tReturn=tReturn-0.04;
                elseif ratio>1.1
                    xTargetPos=xTargetPos-0.04;
                end
                xVReturn=(xTargetPos-(hitPosition(1,1)+cooDist))/tReturn;
                yVReturn=(yTargetPos-hitPosition(1,2))/tReturn;
                zVReturn=(zTargetPos-hitPosition(1,3)+AIPl.g/2*tReturn^2)/tReturn;
                
                vIn=[V(1), V(2), hitZVel];
                vOut=[xVReturn, yVReturn, zVReturn];
                ratio=norm(vOut)/norm(vIn);
            end
            
            normal=vOut - vIn;
            normalizedNomral=normal/norm(normal);

            addVel=normal'-(-(2*(dot(normalizedNomral',vIn')*normalizedNomral')));

            pitch_=asin(normalizedNomral(3));
            yaw_=atan(normalizedNomral(2)/normalizedNomral(1));

            %% Calculate the Direction 
            if(hitPosition(1,2)<=0)
            % Forehand
                % If Roll=90 Pitch and Yaw are interchanged
                roll=90*pi/180;
                pitch=yaw_;
                yaw=pitch_;
            elseif(hitPosition(2)>0)
            % Backhand
                % If Roll=-90 Pitch and Yaw are interchanged
                roll=-90*pi/180;
                pitch=-yaw_;
                yaw=-pitch_;
            else
                warning('Something went wrong!')
                roll=0;
                pitch=0;
                yaw=0;
            end

            hitPosition(1,3)= hitPosition(1,3)-AIPl.Robot.Foundation(4);

        end
        
        endDirection=[hitPosition'; roll; pitch; yaw];
        PaddleVec=[addVel(1);addVel(2);addVel(3);0;0;0];
        endTime=tHitPos+startTime;
        end 
        function [T, InitialAngles]=GetTandInitialAngles(AIPl, Direction)
            %% Calculate T and Get Initial Angels
            % T = End Position Matrix
            T=transl(Direction(1), Direction(2), Direction(3))*...
            trotx(Direction(4))*troty(Direction(5))*...
            trotz(Direction(6));
            
            InitialAngles=AIPl.FindInitialAngles(Direction(1:3), Direction(4:6)*180/pi);
            InitialAngles=InitialAngles*pi/180;
        end  
        function InitialAngles=FindInitialAngles(AIPl, Position, Orientation)
            %% Description
            % This function searches for the Initial Angles for the Inverse Kinematics
            % Therefore it searches for the Values with the lowest Minimum between the
            % inserted Values and the Given Zero Values

            %% 
            DIFF=[abs(AIPl.Direction0(:,1)-Position(1)),...
                  abs(AIPl.Direction0(:,2)-Position(2)),...
                  abs(AIPl.Direction0(:,3)-Position(3)),...
                  abs(AIPl.Direction0(:,5)-Orientation(2)),...
                  abs(AIPl.Direction0(:,6)-Orientation(3))];
            [~, ind]=min(sum(DIFF,2));
            InitialAngles=AIPl.Angle0(ind(1),:);
        end
        function [q,qd,qdd,TimeVec]=GetMovement(AIPl, endAngles, startTime, endTime, currentAngles, PaddleVec)
            %% Calculate Angle-Position, -Velocity and -Acceleration
            % This function calculates the movement matrices and the
            % according Time vectors
            % For every joint the Angle(q), Angle Velocity(qd) and Angle
            % Acceleration (qdd) are calculated for x-timesteps between
            % start- and endTime
            TimeVec=100;
            % Time
            time=endTime-startTime;
            start1=startTime;
            end1=time*4/5-0.01;
            T1=linspace(start1,end1,TimeVec);
            
            start2=end1+0.01;
            end2=endTime;
            T2=linspace(start2,end2,TimeVec);
           
            start3=endTime+0.01;
            end3=start3+1;
            T3=linspace(start3,end3,TimeVec);
            
            % From Current Position to startDirection
            [q1, qd1, qdd1]=jtraj(currentAngles, endAngles, TimeVec, [0,0,0,0,0,0], [0,0,0,0,0,0]);
            endVel=(AIPl.RobotModel.jacob0(endAngles)\PaddleVec)';
            % When Ball is out of Range
            if(endVel(1)>1000 || endVel(2)>1000 || endVel(3)>1000 || endVel(4)>1000 || endVel(5)>1000 || endVel(6)>1000 )
                 warning('Maybe I can''t get this Ball. My Velocities were over 1000. Set them all to 0')
                 endVel=[0 0 0 0 0 0];
            end
            [q2, qd2, qdd2]=jtraj(endAngles, endAngles, TimeVec, [0,0,0,0,0,0], endVel);
            [q3, qd3, qdd3]=jtraj(endAngles, endAngles, TimeVec, endVel, [0,0,0,0,0,0]);
            q=[q1;q2;q3];
            qd=[qd1;qd2;qd3];
            qdd=[qdd1;qdd2;qdd3];
            TimeVec=[T1,T2,T3];
%             q=[q2;q3];
%             qd=[qd2;qd3];
%             qdd=[qdd2;qdd3];
%             TimeVec=[T2,T3];
            
        end
        function update(AIPl, P, V, LastP, LastV , currentTime, currentAngles, currentVelocity)
        %% Update the Player
        % If necesessary calculate trajectory or calculate the
        % configuration in which the Robot has to be set in the next step
        % The calculation of the torque does not work so far in Matlab 2016
        % stand: 21.08.2016
            % Asked in forum for solution:
            % https://groups.google.com/forum/#!topic/robotics-tool-box/YS4oqLymAYk
            % R2Torque=Robot1Model.rne(RightPlayer.Q,RightPlayer.QD,RightPlayer.QDD);
            %%
            % First adjust the V, P, LastV and LastP
            [P,V,LastP,LastV]=AIPl.AdjustPositionAndVelocity(P,V,LastP,LastV);
            if (LastV(1)>0 && V(1)<0) || (LastV(1)<0 && V(1)>0)
                AIPl.PredictEnd=false;
            end
            % If necessary calculate the trajectory to hit the ball
            if(V(1)<0 && P(1) <= 1 && AIPl.PredictEnd==false)
                AIPl.PredictEnd=true;
                [endDirection, endTime,  PaddleVec]...
                      = Calc_Ball_Trajectory(AIPl, P, V, currentTime);
                [T, InitialAngles]=AIPl.GetTandInitialAngles(endDirection);
                endAngles=AIPl.RobotModel.ikunc(T,InitialAngles);
                [q,qd,qdd,TimeVec]=AIPl.GetMovement(endAngles, currentTime, endTime, currentAngles, PaddleVec);
                AIPl.q=q;
                AIPl.qd=qd;
                AIPl.qdd=qdd;
                AIPl.TimeVec=TimeVec;
                if strcmp(AIPl.position,'left')
                    set_param('Pong3D/R1SetAngle', 'Value', ['[',num2str(AIPl.q(1,:)),']'])
                    set_param('Pong3D/R1SetVelocity', 'Value', ['[', num2str(AIPl.qd(1,:)),']'])
                elseif strcmp(AIPl.position,'right')
                    set_param('Pong3D/R2SetAngle', 'Value', ['[',num2str(AIPl.q(1,:)),']'])
                    set_param('Pong3D/R2SetVelocity', 'Value', ['[', num2str(AIPl.qd(1,:)),']'])
                end
            % If the trajectory to hit the ball should be calculated but 
            % V(1)==0, then just skip the update
            elseif(V(1)==0 && P(1) <= 1 && AIPl.PredictEnd==false) 
                disp('V(1)=0! No calculation of the Hit Trajectory! Skip!')
            % If necessary, calculate the trajectory to get from current
            % position to initial positon
            elseif(V(1)>=0 && P(1)>=-1 && AIPl.PredictEnd==false)
                AIPl.PredictEnd=true;
                [q,qd,qdd,TimeVec]=AIPl.GetBack(currentAngles , currentVelocity, currentTime);
                AIPl.q=q;
                AIPl.qd=qd;
                AIPl.qdd=qdd;
                AIPl.TimeVec=TimeVec;
                if strcmp(AIPl.position,'left')
                    set_param('Pong3D/R1SetAngle', 'Value', ['[',num2str(AIPl.q(1,:)),']'])
                    set_param('Pong3D/R1SetVelocity', 'Value', ['[', num2str(AIPl.qd(1,:)),']'])
                elseif strcmp(AIPl.position,'right')
                    set_param('Pong3D/R2SetAngle', 'Value', ['[',num2str(AIPl.q(1,:)),']'])
                    set_param('Pong3D/R2SetVelocity', 'Value', ['[', num2str(AIPl.qd(1,:)),']'])
                end
            % Get the next to set the robot to
            else
                [Q,QD,QDD]=AIPl.GetQs(AIPl.q,AIPl.qd,AIPl.qdd,AIPl.TimeVec,currentTime);
                if strcmp(AIPl.position,'left')
                    set_param('Pong3D/R1SetAngle', 'Value', ['[',num2str(Q),']'])
                    set_param('Pong3D/R1SetVelocity', 'Value', ['[', num2str(QD),']'])
                elseif strcmp(AIPl.position,'right')
                    set_param('Pong3D/R2SetAngle', 'Value', ['[',num2str(Q),']'])
                    set_param('Pong3D/R2SetVelocity', 'Value', ['[', num2str(QD),']'])
                end
                if isnan(Q)
                    if strcmp(AIPl.position,'left')
                        set_param('Pong3D/R1SetAngle', 'Value', ['[',num2str(AIPl.q(end,:)),']'])
                        set_param('Pong3D/R1SetVelocity', 'Value', ['[', num2str(AIPl.qd(end,:)),']'])
                    elseif strcmp(AIPl.position,'right')
                        set_param('Pong3D/R2SetAngle', 'Value', ['[',num2str(AIPl.q(end,:)),']'])
                        set_param('Pong3D/R2SetVelocity', 'Value', ['[', num2str(AIPl.qd(end,:)),']'])
                    end
                end
            end
        end    
    end
    methods(Static)
        function [q,qd,qdd, TimeVec]=GetBack(currentAngles , currentVel, currentTime)
        %% Calculate the way back
        % This function calculates the trajectory from the current Position
        % to the Initial Positon which is set to [0 90 0 0 0 0]
            TimeSteps=100;
            start1=currentTime;
            end1=currentTime+0.1;
            TimeVec1=linspace(start1, end1, TimeSteps);
            [q1, qd1, qdd1]=jtraj(currentAngles, currentAngles, TimeVec1, currentVel, [0,0,0,0,0,0]);
            
            start2=end1+0.01;
            end2=start2+1;
            TimeVec2=linspace(start2, end2, TimeSteps);
            [q2, qd2, qdd2]=jtraj(currentAngles, [0,pi/2,0,0,0,0], TimeVec2, [0,0,0,0,0,0], [0,0,0,0,0,0]);
            TimeVec=[TimeVec1,TimeVec2];
            q=  [q1;q2];
            qd= [qd1; qd2];
            qdd=[qdd1; qdd2];
        end
        function [Q,QD,QDD]=GetQs(q,qd,qdd,TimeVec,time)
            %% Angle-Position, -Velocity and -Acceleration for time t
            Q=zeros(1,6);
            QD=zeros(1,6);
            QDD=zeros(1,6);
            if time==0
                for i=1:6
                    Q(i)=    q(1,i);
                    QD(i)=  qd(1,i);
                    QDD(i)=qdd(1,i);
                end
            else
                for i=1:6
                    Q(i)=  interp1(TimeVec,q(:,i)  ,time);
                    QD(i)= interp1(TimeVec,qd(:,i) ,time);
                    QDD(i)=interp1(TimeVec,qdd(:,i),time);
                end
            end
        end
    end
end
