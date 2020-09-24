clc
clear
tic
%%Check the error in the trim point. 

load equations_of_motion.mat
load eqm_points.mat
toc

   %States and related variables
    syms u v w p q r p_d q_d r_d b1s b1c b2s b2c b1s_d b1c_d b2s_d b2c_d phi_L th_L phi_L_d th_L_d
    
    %Cable length
    syms L 
    %Euler angles, from PayloadEqn.mat
    syms phi_euler th_euler psi_euler
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Control inputs as variables
    syms th0 th1s th1c th2s th2c del_elevator del_rudder
    
   toc
   
   X_states=[u, v, w, p, q, r, phi_euler, th_euler, psi_euler, b1s, b1c, b2s, b2c, phi_L, th_L, phi_L_d, th_L_d, th0, th1s, th1c, th2s, th2c, del_elevator, del_rudder];

   resultant_x_dot=double(subs(x_EOM,X_states, e_point))