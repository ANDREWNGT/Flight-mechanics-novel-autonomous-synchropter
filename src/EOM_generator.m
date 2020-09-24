clc
clear
fprintf("Running Script 1: EOM_generator now.")
load external_forces.mat %Contains resultant moments and forces, flap equations.
load design_constants.mat


%States and related variables
syms u v w p q r p_d q_d r_d  phi_L th_L phi_L_d th_L_d

%Cable length
syms L
%Euler angles, from PayloadEqn.mat
syms phi_euler th_euler psi_euler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Control inputs as variables
syms th0 th1s th1c th2s th2c del_elevator del_rudder


%% Set of non-linear equations
X_states_studied=[u, v, w, p, q, r, phi_euler, th_euler, psi_euler, phi_L, th_L, phi_L_d, th_L_d];
u_control_inputs=[th0, th1s, th1c, th2s, th2c, del_elevator, del_rudder];
x_EOM=sym('X_EOM',[13 1]);

x_EOM(1)=subs(x_EOM(1),-(w*q-v*r)+X_total/m_aircraft-g*sin(th_euler));
x_EOM(2)=subs(x_EOM(2),-(u*r-w*p)+Y_total/m_aircraft-g*cos(th_euler)*sin(phi_euler));
x_EOM(3)=subs(x_EOM(3),-(v*p-u*q)+Z_total/m_aircraft+g*cos(th_euler)*cos(phi_euler));
x_EOM(4)=subs(x_EOM(4),I_xx^-1*(L_total+q*r*(I_yy-I_zz)));
x_EOM(5)=subs(x_EOM(5),I_yy^-1*(M_total+p*r*(I_zz-I_xx)));
x_EOM(6)=subs(x_EOM(6),I_zz^-1*(N_total+p*q*(I_xx-I_yy)));
x_EOM(7)=subs(x_EOM(7),p+q*sin(phi_euler)*tan(th_euler)+r*cos(phi_euler)*tan(th_euler));
x_EOM(8)=subs(x_EOM(8),q*cos(phi_euler)-r*sin(phi_euler));
x_EOM(9)=subs(x_EOM(9),q*sin(phi_euler)*sec(th_euler)+r*cos(phi_euler)*sec(th_euler));
x_EOM(10)=subs(x_EOM(10),phi_L_d);
x_EOM(11)=subs(x_EOM(11),th_L_d);
x_EOM(12)=subs(x_EOM(12),phi_L_dd);
x_EOM(13)=subs(x_EOM(13),th_L_dd);

save equations_of_motion.mat x_EOM

fprintf('Equations of motion generated. Inspect equations_of_motion.mat')

