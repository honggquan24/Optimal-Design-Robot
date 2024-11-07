%__________________________________________________________________     _%
%  Optimization Algorithms for Inverse Kinematics of Robots with MATLAB Source Code
%  Developed in MATLAB R2016b                                            %
%  Author and programmer: Hazim Nasir Ghafil                             %
%                                                                        %
%         e-Mail: hazimn.bedran@uokufa.edu.iq                            %
%                 hazimbedran@gmail.com                                  %
%                                                                        %
% kindly, cite the code as :
% Ghafil H.N., J?rmai K. (2021) Optimization Algorithms for Inverse 
%Kinematics of Robots with MATLAB Source Code. 
% In: J?rmai K., Voith K. (eds) Vehicle and Automotive Engineering 3. VAE 2020.
%Lecture Notes in Mechanical Engineering. Springer, Singapore. 
%https://doi.org/10.1007/978-981-15-9529-5_40

function obj = cost(sol)
t1 = sol(1);
t2 = sol(2);
t3 = sol(3);
t4= sol(4);
t5= sol(5);
[x,y,z] = Forward(t1,t2,t3,t4,t5);
% Cartesian point coordinate [x,x,z]
v = [495,0,255.55];
% Objective function
obj = sqrt((x-v(1))^2+(y-v(2))^2+(z-v(3))^2); 
end