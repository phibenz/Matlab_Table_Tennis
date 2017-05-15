%% Environment
g=9.81;
%% Robot 1
%Joint
J1_field1='Initial_Angle';      J1_value1=[0  90 0  0  0  0];
J1_field2='Set_Angle';          J1_value2=[0  90*pi/180  0  0  0  0];
J1_field22='Set_Velocity';      J1_value22=[0 0 0 0 0 0];
J1_field23='Set_Acceleration';  J1_value23=[0 0 0 0 0 0];
J1_field3='Upper_Bound';        J1_value3=[ 170  190  210  185  120  350];
J1_field4='Lower_Bound';        J1_value4=[-170 -45  -66  -185 -120 -350];
J1_field5='Angle_Range';        J1_value5=[J1_value3(1)-J1_value4(1)...
                                        J1_value3(2)-J1_value4(2)...
                                        J1_value3(3)-J1_value4(3)...
                                        J1_value3(4)-J1_value4(4)...
                                        J1_value3(5)-J1_value4(5)...
                                        J1_value3(6)-J1_value4(6)];
J1_field6='Contact_Stiffness';  J1_value6=[1e3 1e3 1e3 1e3 1e3 1e3];
J1_field7='Contact_Damping';    J1_value7=[10.0 10.0 10.0 10.0 10.0 10.0];
Joint=struct(J1_field1,J1_value1,J1_field2,J1_value2,J1_field22,J1_value22,...
    J1_field23,J1_value23,J1_field3,J1_value3,...
    J1_field4,J1_value4,J1_field5,J1_value5,J1_field6,J1_value6,J1_field7,J1_value7);

clear J1_field1 J1_value1 J1_field2 J1_value2 J1_field3 J1_value3...
      J1_field4 J1_value4 J1_field5 J1_value5 J1_field6 J1_value6...
      J1_field7 J1_value7 J1_field22 J1_value22 J1_field23 J1_value23 

R1_field1='Joint';              R1_value1=Joint;
R1_field2='Foundation';         R1_value2=[10 0.3 0.3 0.76 0.3*0.3*0.76];    % m a b h Vol
% Element Structure:                            m     d     a        Vol 
R1_field3='Element1';           R1_value3=[6.2752639   0.4   0.025 6.2752639];
R1_field4='Element2';           R1_value4=[9.1146006   0     0.455 9.1146006];
R1_field5='Element3';           R1_value5=[2.8742129   0     0.035 2.8742129];
R1_field6='Element4';           R1_value6=[2.9917923   0.420 0     2.9917923];
R1_field7='Element5';           R1_value7=[0.44205811  0     0     0.44205811];
R1_field8='Element6';           R1_value8=[0.013994950 0.08  0     0.013994950];
R1_field9='Paddle';             R1_value9=[0.0775 0.03 100]; % r h rho
R1_field10='Mass';              R1_value10=R1_value3(1)+R1_value4(1)+R1_value5(1)+ ...
                                           R1_value6(1)+R1_value7(1)+R1_value8(1);
R1_field11='Distance_Table';    R1_value11=[0.3 0 0];          % x,y,z
R1_field12='Number';            R1_value12=1;

Robot1=struct(R1_field1,R1_value1,R1_field2,R1_value2,R1_field3,R1_value3,...
    R1_field4,R1_value4, R1_field5,R1_value5,R1_field6,R1_value6,...
    R1_field7,R1_value7,R1_field8,R1_value8,R1_field9,R1_value9,R1_field10,R1_value10,...
    R1_field11,R1_value11,R1_field12,R1_value12);

clear R1_field1 R1_value1 R1_field2 R1_value2 R1_field3 R1_value3...
      R1_field4 R1_value4 R1_field5 R1_value5 R1_field6 R1_value6...
      R1_field7 R1_value7 R1_field8 R1_value8 R1_field9 R1_value9 R1_field10 R1_value10...
      R1_field11 R1_value11 R1_field12 R1_value12

  
%% Robot2

J2_field1='Initial_Angle';     J2_value1=[0  90 0  0  0  0];
J2_field2='Set_Angle';         J2_value2=[0  90*pi/180  0  0  0  0];
J2_field22='Set_Velocity';     J2_value22=[0 0 0 0 0 0];
J2_field23='Set_Acceleration'; J2_value23=[0 0 0 0 0 0];
J2_field3='Upper_Bound';       J2_value3=[ 170  190  210  185  120  350];
J2_field4='Lower_Bound';       J2_value4=[-170 -45  -66  -185 -120 -350];
J2_field5='Angle_Range';       J2_value5=[J2_value3(1)-J2_value4(1)...
                                          J2_value3(2)-J2_value4(2)...
                                          J2_value3(3)-J2_value4(3)...
                                          J2_value3(4)-J2_value4(4)...
                                          J2_value3(5)-J2_value4(5)...
                                          J2_value3(6)-J2_value4(6)];
J2_field6='Contact_Stiffness'; J2_value6=[1e3 1e3 1e3 1e3 1e3 1e3];
J2_field7='Contact_Damping';   J2_value7=[10.0 10.0 10.0 10.0 10.0 10.0];
Joint=struct(J2_field1,J2_value1,J2_field2,J2_value2,J2_field22,J2_value22,...
    J2_field23,J2_value23, J2_field3,J2_value3,...
    J2_field4,J2_value4,J2_field5,J2_value5,J2_field6,J2_value6,J2_field7,J2_value7);

clear J2_field1 J2_value1 J2_field2 J2_value2 J2_field3 J2_value3...
      J2_field4 J2_value4 J2_field5 J2_value5 J2_field6 J2_value6...
      J2_field7 J2_value7 J2_field22 J2_value22 J2_field23 J2_value23

R2_field1='Joint';            R2_value1=Joint;
R2_field2='Foundation';       R2_value2=[10 0.3 0.3 0.76 0.3*0.3*0.76];    % m a b h Vol
% Element Structure:                            m       d     a        Vol 
R2_field3='Element1';           R2_value3=[6.2752639   0.4   0.025 6.2752639];
R2_field4='Element2';           R2_value4=[9.1146006   0     0.455 9.1146006];
R2_field5='Element3';           R2_value5=[2.8742129   0     0.035 2.8742129];
R2_field6='Element4';           R2_value6=[2.9917923   0.420 0     2.9917923];
R2_field7='Element5';           R2_value7=[0.44205811  0     0     0.44205811];
R2_field8='Element6';           R2_value8=[0.013994950 0.08  0     0.013994950];
R2_field9='Paddle';             R2_value9=[0.0775 0.03 100]; % r h rho
R2_field10='Mass';              R2_value10=R2_value3(1)+R2_value4(1)+R2_value5(1)+ ...
                                           R2_value6(1)+R2_value7(1)+R2_value8(1);
R2_field11='Distance_Table';    R2_value11=[0.3 0 0];          % x,y,z
R2_field12='Number';            R2_value12=2;
R2_field13='SetPaddlePosition'; R2_value13=[0.1 0.1 1 10 10];

Robot2=struct(R2_field1,R2_value1,R2_field2,R2_value2,R2_field3,R2_value3,...
    R2_field4, R2_value4, R2_field5,R2_value5,R2_field6,R2_value6,...
    R2_field7, R2_value7, R2_field8, R2_value8, R2_field9, R2_value9, R2_field10, R2_value10,...
    R2_field11, R2_value11, R2_field12, R2_value12);

clear R2_field1 R2_value1 R2_field2 R2_value2 R2_field3 R2_value3...
      R2_field4 R2_value4 R2_field5 R2_value5 R2_field6 R2_value6...
      R2_field7 R2_value7 R2_field8 R2_value8 R2_field9 R2_value9 R2_field10 R2_value10...
      R2_field11 R2_value11 R2_field12 R2_value12

%% Table

T_field1='Dimensions';   T_value1=[2.74 1.525 0.06 1000];  % Length, Width, Thickness, Density 
T_field2='Height';       T_value2=0.76;
Table=struct(T_field1,T_value1,T_field2,T_value2);

clear T_field1 T_value1 T_field2 T_value2;

%% Ball

B_field1='Radius';              B_value1=0.02;
B_field2='Volume';              B_value2=4/3*pi*B_value1^3;
B_field3='Density';             B_value3=2.7/(1000*B_value2);
B_field4='Initial_Position';    B_value4=[1 -0.5 1.3]; % x,y,z
B_field5='Initial_Velocity';    B_value5=[-3 0 1]; % vx, vy, vz
Ball=struct(B_field1,B_value1,B_field2,B_value2,B_field3,B_value3,...
    B_field4,B_value4,B_field5,B_value5);

clear B_field1 B_value1 B_field2 B_value2 B_field3 B_value3 ...
    B_field4 B_value4 B_field5 B_value5

%% Net

N_field1='Dimensions';  N_value1=[1.525, 0.001, 0.1525, 1000]; %Length, Width, Height, Density
Net=struct(N_field1,N_value1);

clear N_field1 N_value1

%% PMDC

PMDC_field1='J';            PMDC_value1=1.55*10^(-3); % Rotor Inertia                   kg*m^2 
PMDC_field2='B';            PMDC_value2=0.03; % Short circuit damping (viscous friction)N*m*s/rad 
PMDC_field3='kt';           PMDC_value3=0.067;  % Torque constant                       N*m/A
PMDC_field4='kb';           PMDC_value4=PMDC_value3; %  Back EMF constant               N*m/A
PMDC_field5='Ra';           PMDC_value5=0.03; % Armature Resistance                     Ohm
PMDC_field6='La';           PMDC_value6=0.1*10^(-3); % Armature Inductance              Henry
PMDC_field7='Voltage_Range';PMDC_value7=24;
PMDC_field8='Gear_Ratio';   PMDC_value8=1/2;
PMDC=struct(PMDC_field1,PMDC_value1,PMDC_field2,PMDC_value2,PMDC_field3,PMDC_value3,...
    PMDC_field4,PMDC_value4,PMDC_field5,PMDC_value5,PMDC_field6,PMDC_value6,...
    PMDC_field7,PMDC_value7,PMDC_field8,PMDC_value8);

clear PMDC_field1 PMDC_value1 PMDC_field2 PMDC_value2 PMDC_field3 PMDC_value3...
      PMDC_field4 PMDC_value4 PMDC_field5 PMDC_value5 PMDC_field6 PMDC_value6... 
      PMDC_field7 PMDC_value7 PMDC_field8 PMDC_value8

%% Contact Forces

CF_field1='Ball_Table';         CF_value1=[1e10 0];     % Contact Stiffness, Contact Damping
CF_field2='Ball_Net';           CF_value2=[1e7  1e12];  
CF_field3='Ball_Paddle';        CF_value3=[1e10 0];
Contact_Forces=struct(CF_field1,CF_value1,CF_field2,CF_value2,CF_field3,CF_value3);

clear CF_field1 CF_value1 CF_field2 CF_value2 CF_field3 CF_value3

%% Set Paddle Position

R1_SetPaddlePosition=[0.1 0.1 1 10 10];
R2_SetPaddlePosition=[0.1 0.1 1 10 10];