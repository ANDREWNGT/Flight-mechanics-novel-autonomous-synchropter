clc
clear

fprintf("Running last script: state_space_model_main now.")
%% Code to generate the A and B matrices. It is expected that the user inputs equilibrium conditions manually.
% As forces and moments are written as symbolic variables, there is no need to alter these for any flight condition.

load external_forces.mat %Contains resultant moments and forces, flap equations.
load design_constants.mat
load eqm_points.mat
%load equations_of_motion.mat


%States and related variables
syms u v w p q r p_d q_d r_d phi_L th_L phi_L_d th_L_d

%Cable length
syms L
%Euler angles, from PayloadEqn.mat
syms phi_euler th_euler psi_euler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Control inputs as variables
syms th0 th1s th1c th2s th2c del_elevator del_rudder



%% State space formulation, A  Matrix, for the selected equilibrium point

load stability_deriv_part1A.mat
state_dimension=13;
A_matrix=zeros(state_dimension);

A_matrix(1,1)= X_vector_e(1);
A_matrix(1,2)= X_vector_e(2)+r_e;
A_matrix(1,3)= X_vector_e(3)-q_e;
A_matrix(1,4)= X_vector_e(4);
A_matrix(1,5)= X_vector_e(5)-w_e;
A_matrix(1,6)= X_vector_e(6)+v_e;
A_matrix(1,8)= -g *cos(th_euler_e);

for i=10:state_dimension
    A_matrix(1,i)=X_vector_e(i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_matrix(2,1)= Y_vector_e(1)-r_e;
A_matrix(2,2)= Y_vector_e(2);
A_matrix(2,3)= Y_vector_e(3)+p_e;
A_matrix(2,4)= Y_vector_e(4)+w_e;
A_matrix(2,5)= Y_vector_e(5);
A_matrix(2,6)= Y_vector_e(6)-u_e;
A_matrix(2,7)= g*cos(th_euler_e)*cos(phi_euler_e);
A_matrix(2,8)= -g *sin(th_euler_e)*sin(phi_euler_e);

for i=10:state_dimension
    A_matrix(2,i)=Y_vector_e(i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_matrix(3,1)= Z_vector_e(1)+q_e;
A_matrix(3,2)= Z_vector_e(2)-p_e;
A_matrix(3,3)= Z_vector_e(3);
A_matrix(3,4)= Z_vector_e(4)-v_e;
A_matrix(3,5)= Z_vector_e(5)+u_e;
A_matrix(3,6)= Z_vector_e(6)-u_e;
A_matrix(3,7)= g*cos(th_euler_e)*sin(phi_euler_e);
A_matrix(3,8)= g *sin(th_euler_e)*cos(phi_euler_e);

for i=10:state_dimension
    A_matrix(3,i)=Z_vector_e(i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_matrix(4,1)= L_vector_e(1);
A_matrix(4,2)= L_vector_e(2);
A_matrix(4,3)= L_vector_e(3);
A_matrix(4,4)= L_vector_e(4);
A_matrix(4,5)= L_vector_e(5)+r_e*(I_yy-I_zz);
A_matrix(4,6)= L_vector_e(6)-q_e*(I_yy-I_zz);

for i=10:state_dimension
    A_matrix(4,i)=L_vector_e(i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_matrix(5,1)= M_vector_e(1)+r_e*(I_zz-I_xx);
A_matrix(5,2)= M_vector_e(2);
A_matrix(5,3)= M_vector_e(3);
A_matrix(5,4)= M_vector_e(4)+r_e*(I_zz-I_xx);
A_matrix(5,5)= M_vector_e(5);
A_matrix(5,6)= M_vector_e(6)+p_e*(I_zz-I_xx);

for i=10:state_dimension
    A_matrix(5,i)=M_vector_e(i);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_matrix(6,1)= N_vector_e(1)+q_e*(I_xx-I_yy);
A_matrix(6,2)= N_vector_e(2);
A_matrix(6,3)= N_vector_e(3);
A_matrix(6,4)= N_vector_e(4)+q_e*(I_xx-I_yy);
A_matrix(6,5)= N_vector_e(5)+p_e*(I_xx-I_yy);
A_matrix(6,6)= N_vector_e(6);

for i=10:state_dimension
    A_matrix(6,i)=N_vector_e(i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_matrix(7,4)= 1;
A_matrix(7,5)= tan(th_euler_e)*sin(phi_euler_e);
A_matrix(7,6)= cos(phi_euler_e)*tan(th_euler_e);
A_matrix(7,7)= q_e*cos(phi_euler_e)*tan(th_euler_e)-r_e*cos(phi_euler_e)*tan(th_euler_e);
A_matrix(7,8)= q_e*sin(phi_euler_e)*(sec(th_euler_e))^2-r_e*cos(phi_euler_e)*(sec(th_euler_e))^2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_matrix(8,5)= cos(phi_euler_e);
A_matrix(8,6)= -sin(phi_euler_e);
A_matrix(8,7)= -q_e*sin(phi_euler_e)-r_e*cos(phi_euler_e);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A_matrix(9,5)= sin(phi_euler_e)*sec(th_euler_e);
A_matrix(9,6)= cos(phi_euler_e)*sec(th_euler_e);
A_matrix(9,7)= q_e*cos(phi_euler_e)*sec(th_euler_e)-r_e*sin(phi_euler_e)*sec(th_euler_e);
A_matrix(9,8)= tan(th_euler_e)*sec(th_euler_e)*(q_e*sin(phi_euler_e)+r_e*cos(phi_euler_e));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:state_dimension
    
    A_matrix(12,i)=phi_L_vector_e(i);
    A_matrix(state_dimension,i)=th_L_vector_e(i);
end
A_matrix(10,12)=1;
A_matrix(11,state_dimension)=1;
%% B matrix formulation, for selected equilibrium point
load control_deriv_.mat
B_matrix=zeros(state_dimension,7);
for i=1:7
    B_matrix(1,i)=X_control_vector_e(i);
    B_matrix(2,i)=Y_control_vector_e(i);
    B_matrix(3,i)=Z_control_vector_e(i);
    B_matrix(4,i)=L_control_vector_e(i);
    B_matrix(5,i)=M_control_vector_e(i);
    B_matrix(6,i)=N_control_vector_e(i);
    
    B_matrix(12,i)=phi_L_control_vector_e(i);
    B_matrix(state_dimension,i)=th_L_control_vector_e(i);
end
%% Save function
%Outputs to folder (mission_legs). Remember to properly label the .mat
%file.
Folder = cd;
Folder = fullfile(Folder, 'mission_legs');
save(fullfile(Folder, 'state_space_model.mat'), 'A_matrix', 'B_matrix')


