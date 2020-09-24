clc
clear

fprintf("Running script 4: control_deriv now.")
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



syms u_d v_d w_d p_d q_d  r_d  phi_euler_d  th_euler_d  psi_euler_d  phi_L_d  th_L_d  th0_d  th1s_d  th1c_d  th2s_d  th2c_d  del_elevator_d  del_rudder_d

X_states=[u, v, w, p, q, r, phi_euler, th_euler, psi_euler, phi_L, th_L, phi_L_d, th_L_d, th0, th1s, th1c, th2s, th2c, del_elevator, del_rudder];
first_derivatives=[u_d,v_d,w_d, p_d, q_d, r_d, phi_euler_d, th_euler_d, psi_euler_d, phi_L_d, th_L_d, th0_d, th1s_d, th1c_d, th2s_d, th2c_d, del_elevator_d, del_rudder_d];
e_point=[u_e, v_e, w_e, p_e, q_e, r_e, phi_euler_e, th_euler_e, psi_euler_e, phi_L_e, th_L_e, phi_L_d_e, th_L_d_e, th0_e, th1s_e, th1c_e, th2s_e, th2c_e, del_elevator_e, del_rudder_e];

%% Equilibrium point calculations of the unknown states
%Equilibrium point consists of all equilibrium values of all states and control inputs.


controls_dimension=7;
tolerance=10^-1;

%Row 1
X_e_numerical=zeros(1,controls_dimension);
for i=1:6
    
    j=13+i;
    
    perturbation=zeros(1,size(e_point,2));
    perturbation(1,j)=tolerance;
    
    X_e=double(subs(X_total,X_states, e_point));
    X_e_perturbed=double(subs(X_total,X_states, e_point+perturbation));
    
    X_e_numerical(i)=(X_e_perturbed-X_e)/tolerance;
    
    fprintf("Finished %f X derivative.", i)
end


X_control_vector_e=X_e_numerical;
X_control_vector_e(1:7)=X_control_vector_e(1:7)/m_aircraft;
fprintf("X control derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))



tolerance=10^-1;

%Row 2
Y_e_numerical=zeros(1,controls_dimension);
for i=1:7
    j=13+i;
    if i==6
        continue
    else
        
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,j)=tolerance;
        
        Y_e=double(subs(Y_total,X_states, e_point));
        Y_e_perturbed=double(subs(Y_total,X_states, e_point+perturbation));
        
        Y_e_numerical(i)=(Y_e_perturbed-Y_e)/tolerance;
        
        fprintf("Finished %f Y derivative.", i)
        
    end
end

Y_control_vector_e=Y_e_numerical;
Y_control_vector_e(1:7)=Y_control_vector_e(1:7)/m_aircraft;
fprintf("Y control derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))




%Row 3
Z_e_numerical=zeros(1,controls_dimension);
for i=1:6
    j=13+i;
    perturbation=zeros(1,size(e_point,2));
    perturbation(1,j)=tolerance;
    
    Z_e=double(subs(Z_total,X_states, e_point));
    Z_e_perturbed=double(subs(Z_total,X_states, e_point+perturbation));
    
    Z_e_numerical(i)=(Z_e_perturbed-Z_e)/tolerance;
    
    fprintf("Finished %f Z derivative.", i)
    
    
end

Z_control_vector_e=Z_e_numerical;
Z_control_vector_e(1:6)=Z_control_vector_e(1:6)/m_aircraft;
fprintf("Z control derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))


L_e_numerical=zeros(1,controls_dimension);
for i=1:7
    j=13+i;
    if i==6
        continue
    else
        
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,j)=tolerance;
        
        L_e=double(subs(L_total,X_states, e_point));
        L_e_perturbed=double(subs(L_total,X_states, e_point+perturbation));
        
        L_e_numerical(i)=(L_e_perturbed-L_e)/tolerance;
        
        fprintf("Finished %f L derivative.", i)
        
    end
end

L_control_vector_e=L_e_numerical;
L_control_vector_e(1:7)=L_control_vector_e(1:7)/I_xx;
fprintf("L control derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))


%Row 5
M_e_numerical=zeros(1,controls_dimension);
for i=1:6
    j=13+i;
    perturbation=zeros(1,size(e_point,2));
    perturbation(1,j)=tolerance;
    
    M_e=double(subs(M_total,X_states, e_point));
    M_e_perturbed=double(subs(M_total,X_states, e_point+perturbation));
    
    M_e_numerical(i)=(M_e_perturbed-M_e)/tolerance;
    
    fprintf("Finished %f M derivative.", i)
end

M_control_vector_e=M_e_numerical;
M_control_vector_e(1:6)=M_control_vector_e(1:6)/I_yy;
fprintf("M control derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))


%Row 6
N_e_numerical=zeros(1,controls_dimension);
for i=1:7
    if i==6
        continue
    else
        j=13+i;
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,j)=tolerance;
        
        N_e=double(subs(N_total,X_states, e_point));
        N_e_perturbed=double(subs(N_total,X_states, e_point+perturbation));
        
        N_e_numerical(i)=(N_e_perturbed-N_e)/tolerance;
        
        fprintf("Finished %f N derivative.", i)
        
    end
end

N_control_vector_e=N_e_numerical;
N_control_vector_e(1:7)=N_control_vector_e(1:7)/I_zz;
fprintf("N control derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))


%%%%%Rows 10 & 11 are zeros
%%%%%%%%%%%%%%%Rows 12 & 13
phi_L_dd_numerical=zeros(1,controls_dimension);
th_L_dd_numerical=zeros(1,controls_dimension);
for i=1:5
    
    j=13+i;
    perturbation=zeros(1,size(e_point,2));
    perturbation(1,j)=tolerance;
    
    phi_L_dd_e=double(subs(phi_L_dd,X_states, e_point));
    phi_L_dd_perturbed=double(subs(phi_L_dd,X_states, e_point+perturbation));
    phi_L_dd_numerical(i)=(phi_L_dd_perturbed-phi_L_dd_e)/tolerance;
    
    fprintf("Finished %f phi_L_dd derivative.", i)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    th_L_dd_e=double(subs(th_L_dd,X_states, e_point));
    th_L_dd_perturbed=double(subs(th_L_dd,X_states, e_point+perturbation));
    th_L_dd_numerical(i)=(th_L_dd_perturbed-th_L_dd_e)/tolerance;
    
    fprintf("Finished %f phi_L_dd derivative.", i)
end

phi_L_control_vector_e=phi_L_dd_numerical;
th_L_control_vector_e=th_L_dd_numerical;
save control_deriv_.mat X_control_vector_e Y_control_vector_e Z_control_vector_e L_control_vector_e M_control_vector_e N_control_vector_e phi_L_control_vector_e th_L_control_vector_e


