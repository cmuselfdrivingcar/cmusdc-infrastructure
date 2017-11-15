x_true = zeros(22,2);

% for i=1:22
% x_true(i,1) = 0.1*cosd(90*i/22);
% x_true(i,2) = 0.1*sind(90*i/22);
% end

for i=1:22
    x_true(i,1) = 0.1 * i;
    x_true(i,2) = 0.1 * i;
end

% for i=1:22
% x_true(i,1) = 0.1*cosd(180*i/22);
% x_true(i,2) = 0.1*sind(180*i/22);
% end

x_pred_ca = zeros(22,2);

x_pred_ca(1:8, :) = x_true(1:8, :);
for i=1:14
x_pred_ca(i+8,:) = 3*x_pred_ca(i+7,:) - 3*x_pred_ca(i+6,:) + x_pred_ca(i+5,:);
end

x_pred_cv = zeros(18,2);

x_pred_cv(1:4, :) = x_true(1:4, :);
for i=1:14
x_pred_cv(i+4,:) = (i+1)*x_true(4,:) -i*x_true(3,:);
end

plot(x_true(:,1), x_true(:,2), 'r')
hold on

plot(x_pred_cv(:,1), x_pred_cv(:,2), 'b')
hold on

plot(x_pred_ca(:,1), x_pred_ca(:,2), 'g')
