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
function [x,y,z] = Forward(t1,t2,t3,t4,t5)
alpha_1 = 90;        %twist angle for link 1
a_1 = 0;             %distacne between z1 and z2
d_1 = 255.55;        % distance between x1 and x2

alpha_2 = 0;         %twist angle for link 2
a_2 = 190;           %distacne between z2 and z3
d_2 = 0;             %distance between x2 and x3

alpha_3 = 0;         %twist angle for link 3
a_3 = 190;           %distacne between z3 and z4
d_3 = 0;             % distance between x3 and x4

alpha_4 = 90;        %twist angle for link 4
a_4 = 0;       %distacne between z4 and z5
d_4 = 0;         % distance between x4 and x5

alpha_5 = 0;      %twist angle for link 5
a_5 = 0;       %distacne between z5 and z6
d_5 = 115;         % distance between x5 and x6

H_1= HTM(t1,alpha_1,a_1,d_1); % HTM 1
H_2= HTM(t2,alpha_2,a_2,d_2); % HTM 2
H_3= HTM(t3,alpha_3,a_3,d_3); % HTM 3
H_4= HTM(t4,alpha_4,a_4,d_4); % HTM 4
H_5= HTM(t5,alpha_5,a_5,d_5); % HTM 5
TH = H_1*H_2*H_3*H_4*H_5;            % over all HTM   
x=TH(1,4);
y=TH(2,4);
z=TH(3,4);
end