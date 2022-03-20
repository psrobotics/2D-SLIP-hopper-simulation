%2D SLIP simulation
%Shuang Peng, shuangpeng00@gmail.com, Jan 2021

g = 9.81;

%Body mass, leg spring coefficient, leg length, simulation time step
%In standard international units
body_m = 0.1;
leg_k = 500;
leg_l_ori = 0.06;
d_t = 0.01;

%Init state, starts in flight phase
body_x_i = 0;
body_z_i = 0.1;

body_dx_i = 0;
body_dz_i = 0;

%Controller parameter
k_ctr = 0.02;
i_ctr = 0.0005;
vel_err_sum = 0;

%Target velocity
des_vel = 0.3;

t_arr = 0:d_t:4;

%Robot overall state for visualization
%x,z,dx,dz,l,ang,dl,dang
state_combined = [];
des_vel_arr = [];

%Hip joint's target angle
leg_ang = 0;

%Sim 50 cycles
for i=1:50
    
    %Flight phase
    dyn_flight = @(t,s_f) [s_f(3);s_f(4);0;-1*g];
    options_flight = odeset('Events',@(t,s_f) check_stance(s_f,leg_l_ori,leg_ang));
    s_f_0 = [body_x_i; body_z_i; body_dx_i; body_dz_i];
    
    [t_f_tmp, s_f_out] = ode45(dyn_flight,t_arr,s_f_0,options_flight);
    
    f_step_counter = ones(size(t_f_tmp(:,1)));
    state_combined = [state_combined; s_f_out(:,1),... %x
        s_f_out(:,2),... %z
        s_f_out(:,3),... %dx
        s_f_out(:,4),... %dz
        f_step_counter * leg_l_ori,... %l
        f_step_counter * leg_ang,... %ang
        f_step_counter * 0,... %dl
        f_step_counter * 0]; %dang
    des_vel_arr = [des_vel_arr; f_step_counter * des_vel];
    state_combined(end,:) = [];
    des_vel_arr(end) = [];
    
    %Flight -> stance, assuming there's no energy loss during the state change
    x_now = s_f_out(end,1);
    dx_tmp = s_f_out(end,3);
    dz_tmp = s_f_out(end,4);
    leg_l_tmp = leg_l_ori;
    leg_ang_tmp = leg_ang;
    leg_dl_tmp = -1 * sin(leg_ang_tmp) * dx_tmp + cos(leg_ang_tmp) * dz_tmp;
    leg_dang_tmp = -1*(dx_tmp * cos(leg_ang_tmp) + dz_tmp * sin(leg_ang_tmp)) / leg_l_tmp;
    
    %Stance phase
    dyn_stance = @(t,s_s)[s_s(3); s_s(4);
        s_s(1)*s_s(4)^2 - g*cos(s_s(2)) + leg_k/body_m*(leg_l_ori-s_s(1));
        -2/s_s(1)*s_s(3)*s_s(4) + g/s_s(1)*sin(s_s(2))];
    options_stance = odeset('Events',@(t,s_s) check_flight(s_s,leg_l_ori));
    s_s_0 = [leg_l_tmp; leg_ang_tmp; leg_dl_tmp; leg_dang_tmp];
    
    [t_s_tmp, s_s_out] = ode45(dyn_stance,t_arr,s_s_0,options_stance);
    
    s_step_counter = ones(size(t_s_tmp(:,1)));
    
    state_combined(end,:) = [];
    des_vel_arr(end) = [];
    state_combined = [state_combined; x_now*s_step_counter - 1*s_s_out(:,1).*sin(s_s_out(:,2)),... %x
        s_s_out(:,1).*cos(s_s_out(:,2)),... %z
        -1*s_s_out(:,3).*sin(s_s_out(:,2)) - s_s_out(:,1).*cos(s_s_out(:,2)).*s_s_out(:,4),... %dx
        s_s_out(:,3).*cos(s_s_out(:,2)) - s_s_out(:,1).*sin(s_s_out(:,2)).*s_s_out(:,4),... %dz
        s_s_out(:,1),... %l
        s_s_out(:,2),... %ang
        s_s_out(:,3),... %dl
        s_s_out(:,4)]; %dang
    des_vel_arr = [des_vel_arr; s_step_counter * des_vel];
    
    %Stance -> flight
    body_z_i = leg_l_ori*cos(s_s_out(end,2));
    body_dx_i = -1*s_s_out(end,3)*sin(s_s_out(end,2)) - leg_l_ori*cos(s_s_out(end,2))*s_s_out(end,4);
    body_dz_i = s_s_out(end,3)*cos(s_s_out(end,2)) - leg_l_ori*sin(s_s_out(end,2))*s_s_out(end,4);
    body_x_i = state_combined(end,1);
    
    state_combined(end,:) = [];
    des_vel_arr(end) = [];
    
    %Calcuate foot's neutral point
    stance_pos = 0.5 * t_s_tmp(end) * body_dx_i;
    vel_err = body_dx_i - des_vel;
    vel_err_sum = vel_err_sum + vel_err;
    
    %Raibert style controller
    x_foot_pos_tar = stance_pos + k_ctr * vel_err + i_ctr * vel_err_sum;
    leg_ang = asin(x_foot_pos_tar/leg_l_ori);
    
    if i>25
        des_vel = -0.25;
    end
    
end

vis(state_combined, des_vel_arr);

%ODE event detection, when changing to stance phase
function [val, is_end, dir] = check_stance(s_f,leg_l_ori,leg_ang_touchdown)
val = s_f(2) - leg_l_ori * cos(leg_ang_touchdown)+ max(s_f(4),0);
is_end = 1;
dir = [];
end

%ODE event detection, when changing to flight phase
function [val, is_end, dir] = check_flight(s_s,leg_l_ori)
val = s_s(1) - leg_l_ori + min(s_s(3),0);
is_end = 1;
dir = [];
end

%Visualization
function [] = vis(state, des_vel)

x = state(:,1);
z = state(:,2);
l = state(:,5);
ang = state(:,6);
x_vel = state(:,3);

%Draw the leg
subplot(5,1,1);
body = patch([0 0],[1 1],[1,0,0]);
leg = patch([0 0],[1 1],[1,1,1]);
ylim(subplot(5,1,1),[0,0.08])
xlim(subplot(5,1,1),[-0.05,0.1])
axis equal;
title('sim');

%Body height
subplot(5,1,2);
title('z');
grid on;
ylim(subplot(5,1,2),[0.02,0.12]);
xlim(subplot(5,1,2),[0,length(x)]);
z_draw = animatedline('Color','b');
ylabel('m');

%X velocity
subplot(5,1,3);
title('x velocity');
grid on;
ylim(subplot(5,1,3),[-0.8,0.8]);
xlim(subplot(5,1,3),[0,length(x)]);
x_vel_draw = animatedline('Color','b');
x_des_vel_draw = animatedline('Color','r');
ylabel('m/s');

%Hip rad
subplot(5,1,4);
title('hip rad');
grid on;
ylim(subplot(5,1,4),[-0.5,0.5]);
xlim(subplot(5,1,4),[0,length(x)]);
phi_draw = animatedline('Color','b');
ylabel('rad');

%Leg length
subplot(5,1,5);
title('leg length');
grid on;
ylim(subplot(5,1,5),[0.03,0.08]);
xlim(subplot(5,1,5),[0,length(x)]);
ll_draw = animatedline('Color','b');
ylabel('m');

%Add all elements
for i = 1:length(x)
    body.XData = [x(i) + 0.01*sin(0:0.1:2*pi)];
    body.YData = [z(i) + 0.01*cos(0:0.1:2*pi)];
    leg.XData = [x(i), x(i)+l(i)*sin(ang(i))];
    leg.YData = [z(i), z(i)-l(i)*cos(ang(i))];
    
    ylim(subplot(5,1,1),[0,0.2])
    xlim(subplot(5,1,1),[-0.05,2])
    
    addpoints(z_draw,i,z(i));
    addpoints(x_vel_draw,i,x_vel(i));
    addpoints(x_des_vel_draw,i,des_vel(i));
    addpoints(phi_draw,i,ang(i));
    addpoints(ll_draw,i,l(i));
    drawnow;
end
end

