function [ P, V ] = RespawnBall(position, Table)
    % This configuration represents, when the ball is spawnd on the right
    % side
    P=zeros(1,3);
    V=zeros(1,3);
    
    % Position
    P(1)=rand()*(1-Table.Dimensions(1)/4)+Table.Dimensions(1)/4;
    P(2)=-(rand()*(Table.Dimensions(2)/2-0.2)+0.1);
    P(3)=Table.Dimensions(3)+rand()+1;
    
    % Velocity
    V(1)=-(rand()+2);
    V(2)=rand();
    V(3)=1+rand();
    
    if strcmp(position,'left')
        P(1)=-P(1);
        P(2)=-P(2);
        
        V(1)=-V(1);
        V(2)=-V(2);
    end
    
    set_param('Pong3D/R1SetAngle', 'Value', '[0 1.5708 0 0 0 0]')
    set_param('Pong3D/R1SetVelocity', 'Value', '[0 0 0 0 0 0]')
    set_param('Pong3D/Subsystem3/R1SetTorque', 'Value', '[0 0 0 0 0 0]')
    
    set_param('Pong3D/R2SetAngle', 'Value', '[0 1.5708 0 0 0 0]')
    set_param('Pong3D/R2SetVelocity', 'Value', '[0 0 0 0 0 0]')
    set_param('Pong3D/Subsystem4/R2SetTorque', 'Value', '[0 0 0 0 0 0]')
    
end