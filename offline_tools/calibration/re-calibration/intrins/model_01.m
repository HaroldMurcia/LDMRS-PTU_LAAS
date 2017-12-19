function [ J ] = model_01( W, DATA_1, DATA_2, DATA_3, DATA_4  )

% LAAS-CNRS: Robotic and Interaction Systems
% SICK LDMRS, Platine Light
% Harold F. MURCIA - November 2017

%   This function contains the model which is used by the optimizer for parameter estimation

    L1z=W(1);
    L2x=W(2);
    L2y=W(3);
    L2z=W(4);
    alpha1=W(5);
    alpha2=W(6);
    alpha3=W(7);
    alpha4=W(8);
    beta_off1=W(9);
    beta_off2=W(10);
    tilt_off=W(11);
    
    r1= DATA_1(:,5);
    r2= DATA_2(:,5);
    r3= DATA_3(:,5);
    r4= DATA_4(:,5);
    
    beta1= DATA_1(:,2);
    beta2= DATA_2(:,2);
    beta3= DATA_3(:,2);
    beta4= DATA_4(:,2);
    
    tilt1= DATA_1(:,1)*pi/180;
    tilt2= DATA_2(:,1)*pi/180;
    tilt3= DATA_3(:,1)*pi/180;
    tilt4= DATA_4(:,1)*pi/180;

%     alpha = [alpha1*ones(S(1),1);alpha2*ones(S(2),1);alpha3*ones(S(3),1);alpha4*ones(S(4),1)];
%     beta_off =  [W(9)+zeros(S(1),1);W(9)+zeros(S(2),1); W(10)+zeros(S(3),1);W(10)+zeros(S(4),1)];
%     X = W(2) - r.*cos(alpha).*sin(beta+beta_off);
%     Y = W(3) - r.*sin(alpha).*sin(tilt+W(11) ) + r.*cos(alpha).*cos(beta+beta_off).*cos(tilt+W(11) );
%     Z = W(1) + W(4) + r.*sin(alpha).*cos(tilt+W(11) ) + r.*cos(alpha).*cos(beta+beta_off).*sin(tilt+W(11) );
%     M=[ones(size(X)), X, Y];
%     K=lscov(M,Z);
%     F1= (K(1) + K(2)*X + K(3)*Y - Z);
    
          
    X_1 = L2x - r1.*cos(alpha1).*sin(beta1+beta_off1);
    Y_1 = L2y - r1.*sin(alpha1).*sin(tilt1+ tilt_off) + r1.*cos(alpha1).*cos(beta1+beta_off1).*cos(tilt1+ tilt_off);
    Z_1 = L1z + L2z + r1.*sin(alpha1).*cos(tilt1+ tilt_off) + r1.*cos(alpha1).*cos(beta1+beta_off1).*sin(tilt1+tilt_off);
    
    X_2 = L2x - r2.*cos(alpha2).*sin(beta2+beta_off1);
    Y_2 = L2y - r2.*sin(alpha2).*sin(tilt2+ tilt_off) + r2.*cos(alpha2).*cos(beta2+beta_off1).*cos(tilt2+ tilt_off);
    Z_2 = L1z + L2z + r2.*sin(alpha2).*cos(tilt2+ tilt_off) + r2.*cos(alpha2).*cos(beta2+beta_off1).*sin(tilt2+tilt_off);
    
    X_3 = L2x - r3.*cos(alpha3).*sin(beta3+beta_off2);
    Y_3 = L2y - r3.*sin(alpha3).*sin(tilt3+ tilt_off) + r3.*cos(alpha3).*cos(beta3+beta_off2).*cos(tilt3+ tilt_off);
    Z_3 = L1z + L2z + r3.*sin(alpha3).*cos(tilt3+ tilt_off) + r3.*cos(alpha3).*cos(beta3+beta_off2).*sin(tilt3+tilt_off);
    
    X_4 = L2x - r4.*cos(alpha4).*sin(beta4+beta_off2);
    Y_4 = L2y - r4.*sin(alpha4).*sin(tilt4+ tilt_off) + r4.*cos(alpha4).*cos(beta4+beta_off2).*cos(tilt4+ tilt_off);
    Z_4 = L1z + L2z + r4.*sin(alpha4).*cos(tilt4+ tilt_off) + r4.*cos(alpha4).*cos(beta4+beta_off2).*sin(tilt4+tilt_off);
    
    X = [X_1;X_2;X_3;X_4];
    Y = [Y_1;Y_2;Y_3;Y_4];
    Z = [Z_1;Z_2;Z_3;Z_4];
    M=[ones(size(X)), X, Y];
    K=lscov(M,Z);
    
    M_1=[ones(size(X_1)), X_1, Y_1];
    M_2=[ones(size(X_2)), X_2, Y_2];
    M_3=[ones(size(X_3)), X_3, Y_3];
    M_4=[ones(size(X_4)), X_4, Y_4];
    K_1=lscov(M_1,Z_1);
    K_2=lscov(M_2,Z_2);
    K_3=lscov(M_3,Z_3);
    K_4=lscov(M_4,Z_4);
    
    F1 = ( K(1) + K(2)*X_1 + K(3)*Y_1 - (K_1(1) + K_1(2)*X_1 + K_1(3)*Y_1) ).^2;
    F2 = ( K(1) + K(2)*X_2 + K(3)*Y_2 - (K_2(1) + K_2(2)*X_2 + K_2(3)*Y_2) ).^2;
    F3 = ( K(1) + K(2)*X_3 + K(3)*Y_3 - (K_3(1) + K_3(2)*X_3 + K_3(3)*Y_3) ).^2;
    F4 = ( K(1) + K(2)*X_4 + K(3)*Y_4 - (K_4(1) + K_4(2)*X_4 + K_4(3)*Y_4) ).^2;

    J= [F1;F2;F3;F4];
end

