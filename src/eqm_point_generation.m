clc
clear
fprintf("Running Script 2: eqm_point_generation now")
load equations_of_motion.mat

%% Equilibrium values. INPUTS REQUIRED.
p_dot_e=10^-3;
q_dot_e=10^-3;
r_dot_e=10^-3;

%KNOWN VARIABLES. INPUT THESE 10 variables below.
%Equilibrium states
u_e=1;
v_e=10^-1;
w_e=10^-1;
p_e=10^-1;
q_e=10^-1;
r_e=10^-1;

phi_euler_e=10^-2; %roll angle
%th_euler_e=10^-2;
psi_euler_e=10^-2; %Yaw angle

phi_L_d_e=10^-3;
th_L_d_e=10^-3;
phi_L_e=10^-1;
% th_L_e=0.62871;
th_L_e=10^-1;

%all in radians

% th1s_e=0.05235;
% th1c_e=0;
% th2s_e=-0.05235;
% th2c_e=-0;
%


tic
syms u v w p q r p_d q_d r_d phi_L th_L phi_L_d th_L_d
% %Euler angles, from PayloadEqn.mat
syms phi_euler
syms th_euler
syms psi_euler
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Control inputs as variables
syms th0 th1s th1c th2s th2c del_elevator del_rudder

unknown_states=[th_euler;th0;th1s;th1c;th2s;th2c;del_elevator;del_rudder]; %Need to solve for this array
known_states=[u; v; w; p; q; r; phi_euler;psi_euler; phi_L;th_L; phi_L_d; th_L_d];
fixed_points=[u_e; v_e; w_e; p_e; q_e; r_e; phi_euler_e; psi_euler_e;phi_L_e;th_L_e; phi_L_d_e; th_L_d_e];

x_EOM_subs=subs(x_EOM, known_states,fixed_points);%Replaces known states with values.
toc
syms x1 x2 x3 x4 x5 x6 x7 x8

variables=sym(zeros(13,1));
% for i=1:13
%     symvar(x_EOM_subs(i))
% end
%% non linear solver
%These 2 lines are done so that we know what is produced by the fsolve()
%step.
placeholder=[x1;x2;x3;x4;x5;x6;x7;x8];
x_EOM_subs=subs(x_EOM_subs, unknown_states,placeholder);%Tags each variable.


fprintf("Appropriate form of x_EOM_subs produced, %s ", datestr(now,'HH:MM:SS.FFF'))
save eqm_equations.mat x_EOM_subs
tic
f = matlabFunction(x_EOM_subs(1),x_EOM_subs(2),x_EOM_subs(3),x_EOM_subs(4),x_EOM_subs(5),x_EOM_subs(6),x_EOM_subs(13),...
    'File','x_EOM_handle', 'Outputs',{'EOM1','EOM2','EOM3','EOM4','EOM5','EOM6','EOM7'});
toc
%%
opt = optimset('Algorithm', 'levenberg-marquardt', 'Display','off');
fprintf("Appropriate function handle produced, %s ", datestr(now,'HH:MM:SS.FFF'))

fun=@x_EOM_handle;
modf = @(x)fun(x(1),x(2), x(3),x(4),x(5),x(6),x(7),x(8));
initial_points=[0;0;0;0;0;0;0;0];
solution=fsolve(modf , initial_points, opt);

th_euler_e=solution(1);
th0_e=solution(2);
th1s_e=solution(3);
th1c_e=solution(4);
th2s_e=solution(5);
th2c_e=solution(6);
del_elevator_e=solution(7);
del_rudder_e=solution(8);

%Function outputs the array e_point.
e_point=[u_e, v_e, w_e, p_e, q_e, r_e, phi_euler_e, th_euler_e, psi_euler_e, phi_L_e, th_L_e, phi_L_d_e, th_L_d_e, th0_e, th1s_e, th1c_e, th2s_e, th2c_e, del_elevator_e, del_rudder_e];

save eqm_points.mat e_point u_e v_e w_e p_e q_e r_e phi_euler_e th_euler_e psi_euler_e phi_L_e th_L_e phi_L_d_e th_L_d_e th0_e th1s_e th1c_e th2s_e th2c_e del_elevator_e del_rudder_e;
load equations_of_motion.mat
% load eqm_points.mat

X_states=[u, v, w, p, q, r, phi_euler, th_euler, psi_euler phi_L, th_L, phi_L_d, th_L_d, th0, th1s, th1c, th2s, th2c, del_elevator, del_rudder];
resultant_x_dot=double(subs(x_EOM,X_states, e_point));
n=norm(resultant_x_dot)
toc
save trim_point_verification.mat resultant_x_dot n