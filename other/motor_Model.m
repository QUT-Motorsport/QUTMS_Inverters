%% Full Car
% Car speed
time_car = [0, 4, 8];

car_max_speed = 106;
car_speed = [0, car_max_speed, 0];

figure
plot(time_car, car_speed)
ylim([0 110])
xlabel('Time (s)')
ylabel('Speed (km/h)')
title('Full car speed graph')
r = car_speed / 3.6;
yyaxis right
plot(time_car, r)
ylabel("Speed (m/s)");

% Car torque
time_car = [0, 4, 4, 8, 8];

car_max_torque = 474.858;
car_inverse_torque = -474.858;
car_torque = [car_max_torque, car_max_torque, car_inverse_torque, car_inverse_torque, 0];
figure
plot(time_car, car_torque)
ylim([-600 600])
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Full car torque graph')

% Car power
time_car = [0, 4, 4, 8];

car_max_power = 69.967;
car_inverse_power = -69.967;
car_power = [0, car_max_power, car_inverse_power, 0];
figure
plot(time_car, car_power)
xlabel('Time (s)')
ylabel('Power (kw)')
title('Full car power graph')

%% Wheel
% Wheel torque
time_car = [0, 4, 4, 8, 8];

wheel_max_torque = 118.715;
wheel_inverse_torque = -118.715;
wheel_torque = [wheel_max_torque, wheel_max_torque, wheel_inverse_torque, wheel_inverse_torque, 0];
figure
plot(time_car, wheel_torque)
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Wheel torque graph')

% Wheel power
time_car = [0, 4, 4, 8];

wheel_max_power = 17.5;
wheel_inverse_power = -17.5;
wheel_power = [0, wheel_max_power, wheel_inverse_power, 0];
figure
plot(time_car, wheel_power)
xlabel('Time (s)')
ylabel('Power (kw)')
title('Wheel power graph')

%% Motor

% Motor speed
time_car = [0, 4, 8];

car_max_speed = 102.14;
car_speed = [0, car_max_speed, 0];

figure
plot(time_car, car_speed)
ylim([0 110])
xlabel('Time (s)')
ylabel('Speed (km/h)')
title('Motor speed graph')
r = car_speed / 3.6;
yyaxis right
plot(time_car, r)
ylabel("Speed (m/s)");


% Motor torque
time_car = [0, 4, 4, 8, 8];

% From the data sheet
motor_max_torque = 626.9;
motor_inverse_torque = -626.9;
motor_torque = [motor_max_torque, motor_max_torque, motor_inverse_torque, motor_inverse_torque, 0];
figure
plot(time_car, motor_torque)
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Motor datasheet torque graph')

time_car = [0, 4, 4, 8, 8];

% From calculations
motor_max_torque = 534.217;
motor_inverse_torque = -534.217;
motor_torque = [motor_max_torque, motor_max_torque, motor_inverse_torque, motor_inverse_torque, 0];
figure
plot(time_car, motor_torque)
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Motor theoratical torque graph')
%ylim([-600 600])

% Motor power
time_car = [0, 4, 4, 8];

motor_max_power = 17.5;
motor_inverse_power = -17.5;
motor_power = [0, motor_max_power, motor_inverse_power, 0];
figure
plot(time_car, motor_power)
xlabel('Time (s)')
ylabel('Power (kw)')
title('Motor power graph')
