clear;
clc;

% Thay đổi các tham số từ bảng modified D-H
th(1) = 0; d(1) = 0; a(1) = 0; alp(1) = 0;
th(2) = 0; d(2) = 0; a(2) = 500; alp(2) = -pi/2;
th(3) = 0; d(3) = 0; a(3) = 1700; alp(3) = 0;
th(4) = 0; d(4) = 2850; a(4) = 180; alp(4) = -pi/2;
th(5) = 0; d(5) = 0; a(5) = 0; alp(5) = pi/2;
th(6) = 0; d(6) = 0; a(6) = 0; alp(6) = -pi/2;

% Định nghĩa các liên kết của robot với tham số mới
L1 = Link([th(1), d(1), a(1), alp(1)], 'modified');
L2 = Link([th(2), d(2), a(2), alp(2)], 'modified');
L3 = Link([th(3), d(3), a(3), alp(3)], 'modified');
L4 = Link([th(4), d(4), a(4), alp(4)], 'modified');
L5 = Link([th(5), d(5), a(5), alp(5)], 'modified');
L6 = Link([th(6), d(6), a(6), alp(6)], 'modified');

% Tạo robot với các liên kết
robot = SerialLink([L1, L2, L3, L4, L5, L6]);
robot.name = '6-dof-robot';

% Hiển thị và mô phỏng robot
robot.plot([0,0,0,0,0,0]);
robot.display();
p = robot.fkine(th);
q = robot.ikine(p);
teach(robot);
