function [ J ] = model_01( W, r, beta, tilt, S )

%% LAAS-CNRS: Robotic and Interaction Systems
% SICK LDMRS, Platine Light
% Harold F. MURCIA - November 2017

%   This function contains the model which is used by the optimizer for parameter estimation

    
%     alpha1     =  -0.031272870583227;
%     alpha2     =  -0.012131387660205;
%     alpha3     =   0.005235987756497;
%     alpha4     =   0.019336219601170;
%     beta_off1   =  -0.011554163109727;
%     beta_off2   =  -0.016255065762611;
%     L1z         =   0.072236184455477;
%     L2x         =   0.100587885615162;
%     L2y         =  -0.074991685813609;
%     L2z         =   0.064653815212933;
%     tilt_off    =  7.532319467863496e-04;
      if(length(W)>3)
          L1z = W(1);
          L2x = W(2);
          L2y = W(3);
          L2z = W(4);
          alpha1 = W(5);
          alpha2 = W(6);
          alpha3 = W(7);
          alpha4 = W(8);
          beta_off1 = W(9);
          beta_off2 = W(10);
          tilt_off = W(11);
          pitch = W(12);
          roll = W(13);
          Loz = W(14); 
      else
        pitch = W(1);
        roll = W(2);
        Loz = W(3); 
        L1z         =  0.100578984193110;
        L2x         = -0.023760000000000;
        L2y         =  0.018797850189242;
        L2z         =  0.083186331806302;
        alpha1     =  -0.026310185433179;
        alpha2     =  -0.007321821152752;
        alpha3     =   0.008640101362527;
        alpha4     =   0.023020415513343;
        beta_off1   =  1.049615948354090e-10;
        beta_off2   =  -1.244528810882754e-11;
        tilt_off    =   -3.962113176508846e-05;
      end
    
    alpha = [alpha1*ones(S(1),1);alpha2*ones(S(2),1);alpha3*ones(S(3),1);alpha4*ones(S(4),1)];
    ba =  beta_off1+zeros(S(1),1);
    bb =  beta_off1+zeros(S(2),1); 
    bc =  beta_off2+zeros(S(3),1); 
    bd =  beta_off2 +zeros(S(4),1);
    beta_off = [ba;bb;bc;bd];

    X = L2x*cos(roll) + L1z*sin(roll) + L2z*sin(roll) + r.*cos(tilt + tilt_off).*sin(alpha).*sin(roll) - r.*cos(alpha).*sin(beta+beta_off).*cos(roll) + r.*sin(tilt + tilt_off).*cos(alpha).*cos(beta+beta_off).*sin(roll);
    Y = L2y*cos(pitch) - L1z*cos(roll)*sin(pitch) - L2z*cos(roll)*sin(pitch) + L2x*sin(pitch)*sin(roll) - r.*sin(alpha).*(sin(tilt + tilt_off).*cos(pitch) + cos(tilt + tilt_off).*cos(roll).*sin(pitch)) + r.*cos(alpha).*cos(beta+beta_off).*(cos(tilt + tilt_off).*cos(pitch) - sin(tilt + tilt_off).*cos(roll).*sin(pitch)) - r.*cos(alpha).*sin(beta+beta_off).*sin(pitch).*sin(roll);
    Z = Loz + L2y*sin(pitch) + L1z*cos(pitch)*cos(roll) + L2z*cos(pitch)*cos(roll) - L2x*cos(pitch)*sin(roll) - r.*sin(alpha).*(sin(tilt + tilt_off).*sin(pitch) - cos(tilt + tilt_off).*cos(pitch).*cos(roll)) + r.*cos(alpha).*cos(beta+beta_off).*(cos(tilt + tilt_off).*sin(pitch) + sin(tilt + tilt_off).*cos(pitch)*cos(roll)) + r.*cos(alpha).*cos(pitch).*sin(beta+beta_off).*sin(roll);

    F1= (Z).^2;
    
    J= [F1];
end

