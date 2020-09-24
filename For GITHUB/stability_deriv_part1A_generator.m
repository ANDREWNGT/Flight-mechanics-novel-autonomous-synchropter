clc
clear
tic
fprintf("Running script 3: stability_deriv_part1A_generator now.")
%% Code to generate the A and B matrices. It is expected that the user inputs equilibrium conditions manually.
% As forces and moments are written as symbolic variables, there is no need to alter these for any flight condition.

load external_forces.mat %Contains resultant moments and forces, flap equations.
load design_constants.mat
load eqm_points.mat
%States and related variables
syms u v w p q r p_d q_d r_d b1s b1c b2s b2c b1s_d b1c_d b2s_d b2c_d phi_L th_L phi_L_d th_L_d

%Cable length
syms L
%Euler angles, from PayloadEqn.mat
syms phi_euler th_euler psi_euler
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Control inputs as variables
syms th0 th1s th1c th2s th2c del_elevator del_rudder

syms u_d v_d w_d p_d q_d  r_d  phi_euler_d  th_euler_d  psi_euler_d  b1s_d  b1c_d  b2s_d  b2c_d  phi_L_d  th_L_d  th0_d  th1s_d  th1c_d  th2s_d  th2c_d  del_elevator_d  del_rudder_d


X_states=[u, v, w, p, q, r, phi_euler, th_euler, psi_euler,phi_L, th_L, phi_L_d, th_L_d, th0, th1s, th1c, th2s, th2c, del_elevator, del_rudder];
first_derivatives=[u_d,v_d,w_d, p_d, q_d, r_d, phi_euler_d, th_euler_d, psi_euler_d, phi_L_d, th_L_d, th0_d, th1s_d, th1c_d, th2s_d, th2c_d, del_elevator_d, del_rudder_d];
e_point=[u_e, v_e, w_e, p_e, q_e, r_e, phi_euler_e, th_euler_e, psi_euler_e,phi_L_e, th_L_e, phi_L_d_e, th_L_d_e, th0_e, th1s_e, th1c_e, th2s_e, th2c_e, del_elevator_e, del_rudder_e];

%% Equilibrium point calculations of the unknown states
%Equilibrium point consists of all equilibrium values of all states and control inputs.

states_dimension=13;
tolerance=10^-1;
X_e_numerical=zeros(1,states_dimension);
for i=1:states_dimension
    
    if i>6 && i<12
        continue
    else
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,i)=tolerance;
        
        X_e=double(subs(X_total,X_states, e_point));
        X_e_perturbed=double(subs(X_total,X_states, e_point+perturbation));
        
        X_e_numerical(i)=(X_e_perturbed-X_e)/tolerance;
        fprintf(" Finished %f derivative.", i)
    end
    
end

X_vector_e=X_e_numerical;
X_vector_e(1:6)=X_vector_e(1:6)/m_aircraft;
fprintf("X stability derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))

Y_e_numerical=zeros(1,states_dimension);
for i=1:states_dimension
    
    if i>6 && i<12
        continue
    else
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,i)=tolerance;
        
        Y_e=double(subs(Y_total,X_states, e_point));
        Y_e_perturbed=double(subs(Y_total,X_states, e_point+perturbation));
        Y_e_numerical(i)=(Y_e_perturbed-Y_e)/tolerance;
        fprintf(" Finished %f derivative.", i)
    end
end
Y_vector_e=Y_e_numerical;
Y_vector_e(1:6)=Y_vector_e(1:6)/m_aircraft;
fprintf("Y stability derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))

Z_e_numerical=zeros(1,states_dimension);
for i=1:states_dimension
    
    if i>6 && i<12
        continue
    else
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,i)=tolerance;
        
        Z_e=double(subs(Z_total,X_states, e_point));
        Z_e_perturbed=double(subs(Z_total,X_states, e_point+perturbation));
        
        Z_e_numerical(i)=(Z_e_perturbed-Z_e)/tolerance;
        fprintf(" Finished %f derivative.", i)
    end
    
end

Z_vector_e=Z_e_numerical;
Z_vector_e(1:6)=Z_vector_e(1:6)/m_aircraft;
fprintf("Z stability derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))
L_e_numerical=zeros(1,states_dimension);
for i=1:states_dimension
    
    if i>6 && i<12
        continue
    else
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,i)=tolerance;
        
        L_e=double(subs(L_total,X_states, e_point));
        L_e_perturbed=double(subs(L_total,X_states, e_point+perturbation));
        
        L_e_numerical(i)=(L_e_perturbed-L_e)/tolerance;
        toc
        fprintf(" Finished %f derivative.", i)
    end
    
end

L_vector_e=L_e_numerical;
L_vector_e(1:6)=L_vector_e(1:6)/I_xx;
fprintf("L stability derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))


M_e_numerical=zeros(1,states_dimension);
for i=1:states_dimension
    
    if i>6 && i<12
        continue
    else
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,i)=tolerance;
        
        M_e=double(subs(M_total,X_states, e_point));
        M_e_perturbed=double(subs(M_total,X_states, e_point+perturbation));
        
        M_e_numerical(i)=(M_e_perturbed-M_e)/tolerance;
        fprintf(" Finished %f derivative.", i)
    end
    
end
M_vector_e=M_e_numerical;
M_vector_e(1:6)=M_vector_e(1:6)/I_yy;
fprintf("M stability derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))

N_e_numerical=zeros(1,states_dimension);
for i=1:states_dimension
    
    if i>6 && i<12
        continue
    else
        perturbation=zeros(1,size(e_point,2));
        perturbation(1,i)=tolerance;
        
        N_e=double(subs(N_total,X_states, e_point));
        N_e_perturbed=double(subs(N_total,X_states, e_point+perturbation));
        
        N_e_numerical(i)=(N_e_perturbed-N_e)/tolerance;
        fprintf(" Finished %f derivative.", i)
    end
    
end

toc
N_vector_e=N_e_numerical;
N_vector_e(1:6)=N_vector_e(1:6)/I_zz;
fprintf("N stability derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))

%%
phi_L_e_numerical=zeros(1,states_dimension);
for i=1:states_dimension
    perturbation=zeros(1,size(e_point,2));
    perturbation(1,i)=tolerance;
    
    phi_L_dd_e=double(subs(phi_L_dd,X_states, e_point));
    phi_L_dd_perturbed=double(subs(phi_L_dd,X_states, e_point+perturbation));
    
    phi_L_e_numerical(i)=(phi_L_dd_perturbed-phi_L_dd_e)/tolerance;
    fprintf(" Finished %f derivative.", i)
    
    
end
phi_L_vector_e=phi_L_e_numerical;
fprintf("phi_L_d stability derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))
th_L_e_numerical=zeros(1,states_dimension);
for i=1:states_dimension
    
    
    perturbation=zeros(1,size(e_point,2));
    perturbation(1,i)=tolerance;
    
    th_L_dd_e=double(subs(th_L_dd,X_states, e_point));
    th_L_dd_perturbed=double(subs(th_L_dd,X_states, e_point+perturbation));
    
    th_L_e_numerical(i)=(th_L_dd_perturbed-th_L_dd_e)/tolerance;
    fprintf(" Finished %f derivative.", i)
end
th_L_vector_e=th_L_e_numerical;
fprintf("th_L_d stability derivatives produced, %s ", datestr(now,'HH:MM:SS.FFF'))
%save stability_deriv_part1A.mat  phi_L_vector_e th_L_vector_e
save stability_deriv_part1A.mat X_vector_e Y_vector_e Z_vector_e L_vector_e M_vector_e N_vector_e phi_L_vector_e th_L_vector_e
