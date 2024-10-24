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
function [T] = HTM(t,alpha,a,d)
T = [cosd(t),-1*sind(t)*cosd(alpha),sind(t)*sind(alpha),a*cosd(t);...
    sind(t),cosd(t)*cosd(alpha),-1*cosd(t)*sind(alpha),a*sind(t);...
    0,sind(alpha),cosd(alpha),d;...
    0,0,0,1];
end