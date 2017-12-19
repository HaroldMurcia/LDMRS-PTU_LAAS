% LAAS-CNRS: Robotic and Interaction Systems
% SICK LDMRS, Platine Light
% Harold F. MURCIA - October 2017

function [ J ] = model_01( W, K, r, beta, tilt, S, Weight )
% input arguments: W=initial parameters, K= initial plane coefficientes,
% r=radial distances, beta= horizontal angles, tilt=Tilt angles, S=data
% layer-markers, Weight=estimation weights.

%   Estimation for intrinsic parameters
    
    alpha = [W(5)*ones(S(1),1);W(6)*ones(S(2),1);W(7)*ones(S(3),1);W(8)*ones(S(4),1)];
    beta_off =  [W(9)+zeros(S(1),1);W(9)+zeros(S(2),1); W(10)+zeros(S(3),1);W(10)+zeros(S(4),1)];
    X = W(2) - r.*cos(alpha).*sin(beta+beta_off);
    Y = W(3) - r.*sin(alpha).*sin(tilt) + r.*cos(alpha).*cos(beta+beta_off).*cos(tilt);
    Z = W(1) + W(4) + r.*sin(alpha).*cos(tilt) + r.*cos(alpha).*cos(beta+beta_off).*sin(tilt);
    M=[ones(size(X)), X, Y];
    K1=lscov(M,Z,Weight);
    
    % Cost function: 
    F1= (K(1) + K(2)*X + K(3)*Y - Z).^2;
    
    J= [F1];
end

