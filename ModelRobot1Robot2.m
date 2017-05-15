%% Robot1
clear L1 Robot1Model
            %Theta    %D                    %A              %Alpha
L1(1)=Link([0     Robot1.Element1(2)  Robot1.Element1(3)  pi/2  0]);
L1(1).I=[7.6206223e-02 5.6269642e-02 5.0080316e-02 -5.3024926e-03 5.1530528e-04 -1.0383390e-05];
L1(1).r=[-1.7031649e-02 -6.7281250e-02 -2.0190342e-03];
L1(1).m=Robot1.Element1(1);

L1(2)=Link([ 0     0                   Robot1.Element2(3)  0     0]);
L1(2).I=[4.2730578e-02 3.0433173e-01 2.8735165e-01 -1.2973557e-03 5.9611628e-05 -1.6703550e-03];
L1(2).r=[-2.5151553e-01  6.9664856e-03  3.9805080e-03];
L1(2).m=Robot1.Element2(1);

L1(3)=Link([-pi/2  0                   Robot1.Element3(3)  pi/2  0]);
L1(3).I=[9.8414109e-03 1.1138140e-02 8.1777602e-03 3.8395011e-06 -5.7681617e-07 -1.4031993e-03];
L1(3).r=[-2.4519872e-02 -2.4633384e-05  1.6069264e-02];
L1(3).m=Robot1.Element3(1);

L1(4)=Link([ 0     Robot1.Element4(2)  0                  -pi/2  0]);
L1(4).I=[2.3729207e-02 5.4707085e-03 2.2867840e-02 -3.8639269e-05 3.7112607e-05 -1.5790305e-05];
L1(4).r=[2.9799054e-04  1.6542350e-01  7.3099779e-05];
L1(4).m=Robot1.Element4(1);

L1(5)=Link([ 0    0                   0                  pi/2  0]);
L1(5).I=[5.6226830e-04 5.2845796e-04 3.1840668e-04 -3.2303329e-07 -1.6625155e-05 3.2007540e-07];
L1(5).r=[-3.0358481e-05 -1.9132014e-03  1.5789952e-02];
L1(5).m=Robot1.Element5(1);

L1(6)=Link([ pi    Robot1.Element6(2)  0                   0     0]);
L1(6).I=[1.7078901e-06 1.7173121e-06 3.0250282e-06 0 0 -8.5608813e-10];
L1(6).r=[4.3141138e-05  0 -7.5196679e-03];
L1(6).m=Robot1.Element6(1);

Robot1Model=SerialLink(L1, 'name', 'Kuka Kr6 R900');
% Set tool
Robot1Model.tool=transl(0, 0, Robot1.Paddle(1));

Angle1=0*pi/180;
Angle2=90*pi/180;
Angle3=0*pi/180;
Angle4=0*pi/180;
Angle5=0*pi/180;
Angle6=0*pi/180;

figure(1)
clf
Robot1Model.plot([Angle1 Angle2 Angle3 Angle4 Angle5 Angle6])
xlabel('x', 'Interpreter', 'Latex')
ylabel('y', 'Interpreter', 'Latex')
zlabel('z', 'Interpreter', 'Latex')


%% Robot2
clear L2 Robot2Model
L2(1)=Link([ 0     Robot2.Element1(2)  Robot2.Element1(3)  pi/2  0]);
L2(1).I=[7.6206223e-02 5.6269642e-02 5.0080316e-02 -5.3024926e-03 5.1530528e-04 -1.0383390e-05];
L2(1).r=[-1.7031649e-02 -6.7281250e-02 -2.0190342e-03];
L2(1).m=Robot2.Element1(1);

L2(2)=Link([ 0     0                   Robot2.Element2(3)  0     0]);
L2(2).I=[4.2730578e-02 3.0433173e-01 2.8735165e-01 -1.2973557e-03 5.9611628e-05 -1.6703550e-03];
L2(2).r=[-2.5151553e-01  6.9664856e-03  3.9805080e-03];
L2(2).m=Robot2.Element2(1);

L2(3)=Link([-pi/2  0                   Robot2.Element3(3)  pi/2  0]);
L2(3).I=[9.8414109e-03 1.1138140e-02 8.1777602e-03 3.8395011e-06 -5.7681617e-07 -1.4031993e-03];
L2(3).r=[-2.4519872e-02 -2.4633384e-05  1.6069264e-02];
L2(3).m=Robot2.Element3(1);

L2(4)=Link([ 0     Robot2.Element4(2)  0                  -pi/2  0]);
L2(4).I=[2.3729207e-02 5.4707085e-03 2.2867840e-02 -3.8639269e-05 3.7112607e-05 -1.5790305e-05];
L2(4).r=[2.9799054e-04  1.6542350e-01  7.3099779e-05];
L2(4).m=Robot2.Element4(1);

L2(5)=Link([ 0     0                   0                   pi/2  0]);
L2(5).I=[5.6226830e-04 5.2845796e-04 3.1840668e-04 -3.2303329e-07 -1.6625155e-05 3.2007540e-07];
L2(5).r=[-3.0358481e-05 -1.9132014e-03  1.5789952e-02];
L2(5).m=Robot2.Element5(1);

L2(6)=Link([ pi    Robot2.Element6(2)  0                   0     0]);
L2(6).I=[1.7078901e-06 1.7173121e-06 3.0250282e-06 0 0 -8.5608813e-10];
L2(6).r=[4.3141138e-05  0 -7.5196679e-03];
L2(6).m=Robot2.Element6(1);

Robot2Model=SerialLink(L2, 'name', 'Robot 2 6DOF');
% Set tool
Robot2Model.tool=transl(0, 0, Robot2.Paddle(1));

% figure(2)
% clf
% Robot2Model.plot([Angle1 Angle2 Angle3 Angle4 Angle5 Angle6])