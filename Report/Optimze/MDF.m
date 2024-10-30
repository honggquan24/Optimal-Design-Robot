% Thiết lập các tham số DH (như đã cấu hình trong ví dụ trước)
clear;
clc;

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

robot = SerialLink([L1, L2, L3, L4, L5, L6]);
robot.name = '6-dof-robot';

% Tạo dữ liệu huấn luyện
num_samples = 1000;
joint_angles = rand(num_samples, 6) * 2 * pi; % Ngẫu nhiên các góc khớp từ 0 đến 2π
positions = zeros(num_samples, 3); % Lưu trữ vị trí (X, Y, Z)

for i = 1:num_samples
    robot.plot(joint_angles(i, :));
    T = robot.fkine(joint_angles(i, :)); % Tính toán động học thuận
    positions(i, :) = T.t'; % Lưu trữ tọa độ cuối (X, Y, Z)
end

% Lưu trữ dữ liệu thành file để dùng cho huấn luyện
save('robot_training_data.mat', 'joint_angles', 'positions');


% Load dữ liệu huấn luyện
load('robot_training_data.mat');

% Chia dữ liệu thành tập huấn luyện và kiểm tra
train_ratio = 0.8;
num_train = round(train_ratio * num_samples);
X_train = joint_angles(1:num_train, :);
Y_train = positions(1:num_train, :);
X_test = joint_angles(num_train+1:end, :);
Y_test = positions(num_train+1:end, :);

% Xây dựng mô hình nơ-ron
layers = [
    featureInputLayer(6)
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(3) % 3 đầu ra (X, Y, Z)
    regressionLayer
];

options = trainingOptions('adam', ...
    'MaxEpochs', 100, ...
    'MiniBatchSize', 32, ...
    'Plots', 'training-progress', ...
    'Verbose', false);

% Huấn luyện mô hình
net = trainNetwork(X_train, Y_train, layers, options);

% Đánh giá mô hình
YPred = predict(net, X_test);
mse = mean((YPred - Y_test).^2, 'all');
fprintf('Mean Squared Error: %.4f\n', mse);


% Dự đoán vị trí của robot cho một góc khớp mới
new_angles = [pi/4, pi/6, pi/3, pi/2, pi/4, pi/3]; % Một ví dụ ngẫu nhiên
predicted_position = predict(net, new_angles);
disp('Vị trí dự đoán của đầu cuối robot:');
disp(predicted_position);
