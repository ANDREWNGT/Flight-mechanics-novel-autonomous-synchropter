clc
clear

close all

load open_loop_analysis.mat
load e_point_analysis

[hover_vector,hover_values]=eig(A_matrix_hover);
[cruise_vector,cruise_values]=eig(A_matrix_cruise);


%% Normalise eigenvector elements to equilibrium states
hover_vector_magnitude=sqrt(real(hover_vector).^2+imag(hover_vector).^2);
normalised_hover_magnitude=zeros(13);
cruise_vector_magnitude=sqrt(real(cruise_vector).^2+imag(cruise_vector).^2);
normalised_cruise_magnitude=zeros(13);
for i=1:13
    for j=1:13
        normalised_hover_magnitude(i,j)=hover_vector_magnitude(i,j)./e_point_hover(j)';
        normalised_cruise_magnitude(i,j)=cruise_vector_magnitude(i,j)./e_point_cruise(j)';
    end
end


eigenvalues_hover=diag(hover_values);
eigenvalues_cruise=diag(cruise_values);


figure(1)
box on 
grid minor
hold on 
hold on

xlabel('Re(z)','interpreter', 'latex', 'fontsize', 30, 'Rotation', 0)
ylabel('Im(z)','interpreter', 'latex', 'fontsize', 30, 'Rotation', 0)
set(gca,'FontSize',30)
title('Root locus plot, hover with payload coupled', 'interpreter', 'latex', 'fontsize', 20, 'Rotation', 0)

hover_root_locus=plot(eigenvalues_hover, 'bo', 'MarkerSize', 10);
plot([0 0], [-100 100], 'k') %Im axis
plot([-8 8], [0 0], 'k') %Im axis
L=legend([hover_root_locus],'Hover with payload');
L.FontSize=20;


figure(2)
box on 
grid minor
hold on 
hold on

xlabel('Re(z)','interpreter', 'latex', 'fontsize', 30, 'Rotation', 0)
ylabel('Im(z)','interpreter', 'latex', 'fontsize', 30, 'Rotation', 0)
set(gca,'FontSize',30)
title('Root locus plot, cruise with payload, coupled', 'interpreter', 'latex', 'fontsize', 20, 'Rotation', 0)
cruise_root_locus=plot(eigenvalues_cruise, 'ro', 'MarkerSize', 10);
plot([0 0], [-100 100], 'k') %Im axis
plot([-8 8], [0 0], 'k') %Im axis
L=legend([cruise_root_locus], 'Cruise with payload');
L.FontSize=20;

figure(3)
box on 
grid minor
hold on 
hold on

xlabel('Re(z)','interpreter', 'latex', 'fontsize', 30, 'Rotation', 0)
ylabel('Im(z)','interpreter', 'latex', 'fontsize', 30, 'Rotation', 0)
set(gca,'FontSize',30)
title('Root locus plot, hover and cruise with payload, significant mode changes. coupled', 'interpreter', 'latex', 'fontsize', 20, 'Rotation', 0)

eigenvalues_changed_hover(1)=eigenvalues_hover(6);
eigenvalues_changed_hover(2)=eigenvalues_hover(7);
eigenvalues_changed_hover(3)=eigenvalues_hover(8);


eigenvalues_changed_cruise(1)=eigenvalues_cruise(8);
eigenvalues_changed_cruise(2)=eigenvalues_cruise(9);
eigenvalues_changed_cruise(3)=eigenvalues_cruise(10);

hover_root_locus=plot(eigenvalues_changed_hover, 'bo', 'MarkerSize', 10);
cruise_root_locus=plot(eigenvalues_changed_cruise, 'ro', 'MarkerSize', 10);
plot([0 0], [-100 100], 'k') %Im axis
plot([-2 2], [0 0], 'k') %Im axis
L=legend([hover_root_locus cruise_root_locus],'Hover with payload', 'Cruise with payload');
L.FontSize=20;


[hover_vector_paper, A_str]=sd_round(hover_vector, 2, 1, 1, 0, '', '');
[cruise_vector_paper, A_str]=sd_round(cruise_vector, 2, 1, 1, 0, '', '');

matrix2latexmatrix(hover_vector_paper, 'eig_hover.tex')
matrix2latexmatrix(cruise_vector_paper, 'eig_cruise.tex')
