frame = (1:20)';

x_true = zeros(20,1);
y_true = zeros(20,1);

x_pred = zeros(20,1);
y_pred = zeros(20,1);

error = zeros(20,2);

for i= 1:20
    x_true(i) = cosd(180*i/20);
    y_true(i) = sind(180*i/20);
end

p_x = polyfit(frame(1:12), x_true(1:12), 3);
p_y = polyfit(frame(1:12), y_true(1:12), 3);

x_pred(1:12) = x_true(1:12);
y_pred(1:12) = y_true(1:12);

for i=12:20
    x_pred(i) = p_x(1) * frame(i) * frame(i) * frame(i) + p_x(2) * frame(i) * frame(i) + p_x(3) * frame(i) + p_x(4);
    y_pred(i) = p_y(1) * frame(i) * frame(i) * frame(i) + p_y(2) * frame(i) * frame(i) + p_y(3) * frame(i) + p_y(4);
    error(i,1) = x_pred(i) - x_true(i);
    error(i,2) = y_pred(i) - y_true(i);
end

true = [x_true(:,1) y_true(:,1)];
pred = [x_pred(:,1) y_pred(:,1)];
error

plot(true(:,1), true(:,2),'b');
hold on;
plot(pred(:,1), pred(:,2),'r');
