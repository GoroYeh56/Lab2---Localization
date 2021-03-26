%tire diameter = 12cm D = 24cm
tire_r = 0.06;%m
tire_d = 0.24;
%% Max speed
max_omega = 2 * pi
max_speed = tire_r*max_omega
% max_speed = 1;
max_angular_vel = 2*max_speed/tire_d

freq = 10;
r = rateControl(freq);
dt = 1/freq;
%setting block
% Command (Goal)
x_c = -5;
y_c = -5;
angle_c = 7*pi/6 ;
theta_c = 0;


A = [5,0,deg2rad(90)];
B = [5,5,deg2rad(120)];
C = [0,5,deg2rad(-150)];
D = [-5,5,deg2rad(-90)];

E = [-5,0,deg2rad(90)];
F = [-5,-5,deg2rad(150)];
G = [0,-5,deg2rad(-150)];
H = [5,-5,deg2rad(-90)];

figure(1);
axis([-8 8 -8 8]);
title('Test Localization Tasks');
xlabel('x');
ylabel('y');
hold on;


Test_goal =[A; B; C; D; E; F; G; H];
NUM_OF_TESTS = 8;
% Plot goal pose.
%for i=1:NUM_OF_TESTS
%   fprintf("%d\n",i);
%   plot(Test_goal(1,(i-1)*3+1),  Test_goal(1,(i-1)*3+2), 'ro') 
%   hold on;
%end
%Test_goal(1,:)
Test_goal(:,1);
Test_goal(:,2)
plot(Test_goal(:,1), Test_goal(:,2), 'ro')
hold on;

x = 0;
y = 0;
theta = -pi/2;
angle = 0;
%% Initial pose
x_i = x;
y_i = y;
angle_i = angle;

x_err = x_c - x;
y_err = y_c - y;
angle_err = angle_c - angle;
%theta_err = theta_c -theta;

k_rho = 2;
k_alpha = 8;
k_beta = -1.5;

counter = 0;
time = 0;



for idx = 1:NUM_OF_TESTS

    x_c = Test_goal(idx,1);
    y_c = Test_goal(idx,2);
    angle_c = Test_goal(idx,3);
    fprintf('Test %d: Goal pose:[%d %d %f] \n', idx,x_c,y_c,angle_c); %X : %f Y : %f ANG : %f TIME : %f COUNTER : %d\n',x,y,angle*180/pi,time,counter)
    x = 0;
    y = 0;
    angle = 0;
    counter = 0;
    
    while(x_err ~= 0 | y_err ~=0 | abs(angle_err*180/pi) > 0.1)
        counter = counter + 1;
        %error calculation block
        x_err = x_c - x;
        y_err = y_c - y;
        angle_err = angle_c -angle;
        %emergency stopping procedure
        if abs(angle_err*180/pi) < 0.1 && abs(x_err) < 0.001 && abs(y_err) < 0.001
            break;
        end

        if abs((angle_c - (2*pi + angle))*180/pi) < 0.1 && abs(x_err) < 0.001 && abs(y_err) < 0.001
            break;
        end

        %theta_err = theta_c -theta;
        %linearisation block
        rho = (x_err^2 + y_err^2)^0.5;
    %     alpha = (atan2(y_err,x_err) - theta);
    %     beta = (-theta - alpha);
        %new version

        % xc : robot_x.
        % xi : initial_x.


        % 1st - quardrum
        if x_c - x_i >= 0 && y_c - y_i >= 0
            alpha = (atan2(y_err,x_err) - angle);
            beta = angle_c - atan2(y_err,x_err);
        end
        % 2nd - quardrum
        if x_c - x_i <= 0 && y_c - y_i >= 0
            alpha = (atan2(y_err,x_err) - angle);
            beta = angle_c + (2*pi - atan2(y_err,x_err));
        end
        % 3rd quardrum
        if x_c - x_i <=0 && y_c - y_i <=0
            alpha = pi + atan2(y_err,x_err) - angle + pi;
            beta = -(pi + atan2(y_err,x_err) - angle_c) + pi;
        end
    %angle > pi
    %     if x_c - x_i <=0 && y_c - y_i <=0
    %         alpha = pi + atan2(y_err,x_err) - angle + pi;
    %         beta = -(pi + atan2(y_err,x_err) + angle_c) + pi ;
    %     end
        % 4th quardrum
        if x_c - x_i >=0 && y_c - y_i <=0
            alpha = pi + atan2(y_err,x_err) - angle + pi;
            beta = -(pi + atan2(y_err,x_err) - angle_c) + pi;
        end

        %angle modification block limit between -pi and pi
        alpha = angle_modifier(alpha);
        beta = angle_modifier(beta);
       % fprintf('RHO : %f ALPHA : %f BETA : %f\n',rho,alpha,beta)
        %velocity generator
        %%%% Command Inputs
        v = k_rho*rho;
        omega = k_alpha*alpha + k_beta*beta;

        % Maximum command input constraint
        if omega > max_angular_vel
            omega = max_angular_vel;
        end

        if omega < -max_angular_vel
            omega = -max_angular_vel;
        end

        if v > max_speed
            v = max_speed;
        end

        if v < -max_speed
            v = -max_speed;
        end
    %     fprintf('V : %f OMEGA : %f\n',v,omega)
        %plant
        %fprintf('X : %f Y : %f ANG : %f TIME : %f COUNTER : %d\n',x,y,angle*180/pi,time,counter)

        %%Update Robot pose
        x = x + v*cos(angle)*dt;
        y = y + v*sin(angle)*dt;

    %    angle = angle + omega*dt;
        angle = angle + omega*dt;
        if(angle<0)
            angle= angle+2*pi;
        end
            %% Calculate theta in 4 different scenario (4 quaters)
        if x_c - x_i >=0 && y_c - y_i >=0
            theta = angle - angle_c;;
        end

        if x_c - x_i <=0 && y_c - y_i >=0
            theta = angle - angle_c;;
        end

        if x_c - x_i <=0 && y_c - y_i <=0
            theta = angle - angle_c + pi;
        end

    %angle_c > pi
    %     if x_c - x_i <=0 && y_c - y_i <=0
    %         theta = angle - angle_c + pi;
    %     end

        if x_c - x_i >=0 && y_c - y_i <=0
            theta = angle - angle_c + pi;
        end

        % Your code goes here
        % Inheritant Counter
        time = r.TotalElapsedTime;
        %figure(1)
        scatter(x,y,'.');
        hold on;

     %   fprintf('X : %f Y : %f ANGLE : %f\n',x,y,angle )

        if counter >= 350
            break;
        end
        % Counter
        time  = time + dt;
        waitfor(r);
    end

    
end


legend('goal pose', 'robot path')
    