function [P,V,currentTime,R1CurrentAngles,R1CurrentVelocity,...
          R2CurrentAngles,R2CurrentVelocity] = ...
                          TransformData(Px, Py, Pz, Vx, Vy, Vz, ...
                                        currentTime,...
                                        R1CurrentAngles, R1CurrentVelocity,...
                                        R2CurrentAngles, R2CurrentVelocity)
                                    
%% Transform and Store Data 
% This function transforms the Data from the Simulation to make it usable
% for the Rest of the calculation

% Position
P=[Px.signals.values, Py.signals.values, Pz.signals.values];
% Velocity
V=[Vx.signals.values, Vy.signals.values, Vz.signals.values];
% current Time
currentTime=currentTime.signals.values;
% Robot1 current Angles and Velocity
R1CurrentAngles=R1CurrentAngles.signals.values;
R1CurrentVelocity=R1CurrentVelocity.signals.values;
% Robot2 current Angles and Velocity
R2CurrentAngles=R2CurrentAngles.signals.values;
R2CurrentVelocity=R2CurrentVelocity.signals.values;

end

