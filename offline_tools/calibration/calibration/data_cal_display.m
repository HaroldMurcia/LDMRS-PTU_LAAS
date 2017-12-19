% LAAS-CNRS: Robotic and Interaction Systems
% SICK LDMRS, Platine Light
% Harold F. MURCIA - October 2017

%%
clear, clc, close all

% loading data for calibration

load data_1.mat
load data_2.mat
load data_3.mat
load data_4.mat
load segmented_data.mat

%% Position 1 (initial)
ang= -Minnie_yaw(1);
off_x = Minnie_POS_XYZ(1,1) - 0;
off_y = Minnie_POS_XYZ(1,2) - 0;
off_z = Minnie_POS_XYZ(1,3) - 0;

T1 = [cos(ang), -sin(ang), 0, off_x;...
      sin(ang),  cos(ang), 0, off_y;...
      0, 0, 1, off_z;...
      0, 0, 0, 1];
  
M1= [data_surf_1(:,12)'; data_surf_1(:,13)'; data_surf_1(:,14)'; ones(1,length(data_surf_1(:,12)))];  
surf_1_0 = T1*M1;

figure(1)
    scatter3(data_surf_1(:,12),data_surf_1(:,13),data_surf_1(:,14),0.1,'.')
    hold on
    scatter3(surf_1_0(1,:),surf_1_0(2,:),surf_1_0(3,:),0.1,'.')
    legend('local frame', 'absolute frame')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('Position 1')


%% Position 2 
ang= -Minnie_yaw(2);
off_x = Minnie_POS_XYZ(2,1) - 0;
off_y = Minnie_POS_XYZ(2,2) - 0;
off_z = Minnie_POS_XYZ(2,3) - 0;

T1 = [cos(ang), -sin(ang), 0, off_x;...
      sin(ang),  cos(ang), 0, off_y;...
      0, 0, 1, off_z;...
      0, 0, 0, 1];
  
M2= [data_surf_2(:,12)'; data_surf_2(:,13)'; data_surf_2(:,14)'; ones(1,length(data_surf_2(:,12)))];  
surf_2_0 = T1*M2;  

figure(2)
    scatter3(data_surf_2(:,12),data_surf_2(:,13),data_surf_2(:,14),0.1,'.')
    hold on
    scatter3(surf_2_0(1,:),surf_2_0(2,:),surf_2_0(3,:),0.1,'.')
    legend('local frame', 'absolute frame')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('Position 2')

%% Position 3 
ang= -Minnie_yaw(3);
off_x = Minnie_POS_XYZ(3,1) - 0;
off_y = Minnie_POS_XYZ(3,2) - 0;
off_z = Minnie_POS_XYZ(3,3) - 0;

T1 = [cos(ang), -sin(ang), 0, off_x;...
      sin(ang),  cos(ang), 0, off_y;...
      0, 0, 1, off_z;...
      0, 0, 0, 1];
  
M3= [data_surf_3(:,12)'; data_surf_3(:,13)'; data_surf_3(:,14)'; ones(1,length(data_surf_3(:,12)))];  
surf_3_0 = T1*M3;  

figure(3)
    scatter3(data_surf_3(:,12),data_surf_3(:,13),data_surf_3(:,14),0.1,'.')
    hold on
    scatter3(surf_3_0(1,:),surf_3_0(2,:),surf_3_0(3,:),0.1,'.')
    legend('local frame', 'absolute frame')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('Position 3')

%% Position 4 
ang= -Minnie_yaw(4);
off_x = Minnie_POS_XYZ(4,1) - 0;
off_y = Minnie_POS_XYZ(4,2) - 0;
off_z = Minnie_POS_XYZ(4,3) - 0;

T1 = [cos(ang), -sin(ang), 0, off_x;...
      sin(ang),  cos(ang), 0, off_y;...
      0, 0, 1, off_z;...
      0, 0, 0, 1];
  
M4= [data_surf_4(:,12)'; data_surf_4(:,13)'; data_surf_4(:,14)'; ones(1,length(data_surf_4(:,12)))];  
surf_4_0 = T1*M4;  

figure(4)
    scatter3(data_surf_4(:,12),data_surf_4(:,13),data_surf_4(:,14),0.1,'.')
    hold on
    scatter3(surf_4_0(1,:),surf_4_0(2,:),surf_4_0(3,:),0.1,'.')
    legend('local frame', 'absolute frame')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title('Position 4')

%% Graphics
figure(5)
    scatter3(surf_1_0(1,:),surf_1_0(2,:),surf_1_0(3,:),0.1,'b.')
    hold on
    scatter3(surf_2_0(1,:),surf_2_0(2,:),surf_2_0(3,:),0.1,'r.')
    scatter3(surf_3_0(1,:),surf_3_0(2,:),surf_3_0(3,:),0.1,'g.')
    scatter3(surf_4_0(1,:),surf_4_0(2,:),surf_4_0(3,:),0.1,'k.')
    legend('From position 1','From position 2','From position 3','From position 4')
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')
    axis([-1.5 2.5 8.85 9.25 0 2])
    grid on


