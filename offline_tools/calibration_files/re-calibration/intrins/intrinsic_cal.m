%% intrinsic re-calibration
%  calibration of intrinsic parameters by using a street scanning on file
%  laas_treet.txt previusly segmented

% LAAS-CNRS: Robotic and Interaction Systems
% SICK LDMRS, Platine Light
% Harold F. MURCIA - November 2017

clear, clc, close all

%% Initial Parameters NOMINAL
    alpha1 = -1.6*pi/180;
    alpha2 = -0.8*pi/180;
    alpha3 = +0.8*pi/180;
    alpha4 = +1.6*pi/180;
    ALPHA = [alpha1,alpha2,alpha3,alpha4];
    L1z  = (10.18 + 3.559*25.4)/1000.0;
    L2z  = (4.4 -0.631 +0.638 + 1.54*2.54)/100.0;
    L2y = (44-25.2)/1000.0;
    L2x = (45-68.76)/1000.0;
    tilt_off= 0;
    beta_off = 0;
    beta_off1 = 0;
    beta_off2 = 0;
    W=[L1z,L2x,L2y,L2z,alpha1,alpha2,alpha3,alpha4,beta_off1,beta_off2, tilt_off];
    lb=[0.075, -0.3, -0.2, 0.08, -2.0*pi/180,-0.8*pi/180,0.3*pi/180,1.1*pi/180, -0.1, -0.1, -2*0.0175];
    ub=[0.2, 0.3, 0.2, 0.15, -1.1*pi/180, -0.3*pi/180, 0.8*pi/180, 1.6*pi/180, 0.1, 0.1, 2*0.0175];
    

%% loading data from laas_street.txt (or other long flat ground profile)
    data = load('........................./re-calibration/intrins/laas_street.txt');
%% spliting data by layer
    echo=data(:,6);
    layer = data(:,4);
    pos_1 = find(layer==0);
    pos_2 = find(layer==1);
    pos_3 = find(layer==2);
    pos_4 = find(layer==3);
    DATA_1=data(pos_1,:);
    DATA_2=data(pos_2,:);
    DATA_3=data(pos_3,:);
    DATA_4=data(pos_4,:);
 
 %% Joining the data
 
    alpha=[alpha1*ones(size(DATA_1(:,1))); alpha2*ones(size(DATA_2(:,1))); alpha3*ones(size(DATA_3(:,1))); alpha4*ones(size(DATA_4(:,1))) ];
    r=[DATA_1(:,5);DATA_2(:,5);DATA_3(:,5);DATA_4(:,5)];
    tilt = [DATA_1(:,1);DATA_2(:,1);DATA_3(:,1);DATA_4(:,1)];
    beta= [DATA_1(:,3);DATA_2(:,3);DATA_3(:,3);DATA_4(:,3)];
    scan = [DATA_1(:,10);DATA_2(:,10);DATA_3(:,10);DATA_4(:,10)];
    pw = [DATA_1(:,7);DATA_2(:,7);DATA_3(:,7);DATA_4(:,7)];
    S=[length(DATA_1(:,1)),length(DATA_2(:,1)),length(DATA_3(:,1)),length(DATA_4(:,1))];
    tilt = tilt*pi/180;
    
%% optimization
    
    X = L2x - r.*cos(alpha).*sin(beta+beta_off);
    Y = L2y - r.*sin(alpha).*sin(tilt+ tilt_off) + r.*cos(alpha).*cos(beta+beta_off).*cos(tilt+ tilt_off);
    Z = L1z + L2z + r.*sin(alpha).*cos(tilt+ tilt_off) + r.*cos(alpha).*cos(beta+beta_off).*sin(tilt+tilt_off);
    
%% Filtering data by pulse widht
        pw = data(:,7);
        plot3(X,Y,pw,'.')
        grid on
        hold on
        pw_u = mean(pw);
        a = 0.75;
        P=find(pw>(pw_u-a) & pw<(pw_u+a));
        plot3(X(P),Y(P),pw(P),'.')
        view(-90,0)
        
        DATA = data(P,:);
        echo=DATA(:,6);
        layer = DATA(:,4);
        pos_1 = find(layer==0);
        pos_2 = find(layer==1);
        pos_3 = find(layer==2);
        pos_4 = find(layer==3);
        DATA_1=DATA(pos_1,:);
        DATA_2=DATA(pos_2,:);
        DATA_3=DATA(pos_3,:);
        DATA_4=DATA(pos_4,:);
        
        alpha=[alpha1*ones(size(DATA_1(:,1))); alpha2*ones(size(DATA_2(:,1))); alpha3*ones(size(DATA_3(:,1))); alpha4*ones(size(DATA_4(:,1))) ];
        r=      [DATA_1(:,5);DATA_2(:,5);DATA_3(:,5);DATA_4(:,5)];
        tilt =  [DATA_1(:,1);DATA_2(:,1);DATA_3(:,1);DATA_4(:,1)];
        beta=   [DATA_1(:,3);DATA_2(:,3);DATA_3(:,3);DATA_4(:,3)];
        scan = [DATA_1(:,10);DATA_2(:,10);DATA_3(:,10);DATA_4(:,10)];
        pw = [DATA_1(:,7);DATA_2(:,7);DATA_3(:,7);DATA_4(:,7)];
        S=[length(DATA_1(:,1)),length(DATA_2(:,1)),length(DATA_3(:,1)),length(DATA_4(:,1))];
        tilt = tilt*pi/180;
        
        X = L2x - r.*cos(alpha).*sin(beta+beta_off);
        Y = L2y - r.*sin(alpha).*sin(tilt+ tilt_off) + r.*cos(alpha).*cos(beta+beta_off).*cos(tilt+ tilt_off);
        Z = L1z + L2z + r.*sin(alpha).*cos(tilt+ tilt_off) + r.*cos(alpha).*cos(beta+beta_off).*sin(tilt+tilt_off);
    
        
    Weight = sqrt(Y/max(Y));
    %Weight = ones(size(Y));
    options = optimoptions(@lsqnonlin,'Display','iter');%'Algorithm','levenberg-marquardt'
    f= @(W)model_01(W, DATA_1, DATA_2, DATA_3, DATA_4 );
    [W2,resnorm,residual,exitflag,output] = lsqnonlin(f,W,lb,ub,options);
    L1z=W2(1);L2x=W2(2);L2y=W2(3);L2z=W2(4);alpha1=W2(5);alpha2=W2(6);alpha3=W2(7);alpha4=W2(8); beta_off1=W2(9); beta_off2=W2(10);tilt_off=W2(11);
    disp('Residual: '); disp(sum(residual.^2))
 %% graphics
    % Nominal
    plot3(X,Y,Z,'.')
    grid on
    hold on

    % calibrated
    alpha = [alpha1*ones(S(1),1);alpha2*ones(S(2),1);alpha3*ones(S(3),1);alpha4*ones(S(4),1)];
    beta_off =  [beta_off1+zeros(S(1),1);beta_off1+zeros(S(2),1); beta_off2+zeros(S(3),1);beta_off2+zeros(S(4),1)];
    Xcal = L2x - r.*cos(alpha).*sin(beta+beta_off);
    Ycal = L2y - r.*sin(alpha).*sin(tilt+tilt_off) + r.*cos(alpha).*cos(beta+beta_off).*cos(tilt+tilt_off);
    Zcal = L1z + L2z + r.*sin(alpha).*cos(tilt+tilt_off) + r.*cos(alpha).*cos(beta+beta_off).*sin(tilt+tilt_off);
    plot3(Xcal,Ycal,Zcal,'.')
    view(-90,0)
    
    M=[ones(size(Xcal)), Xcal, Ycal];
    K1=lscov(M,Zcal,Weight);
    Zplane = K1(1) + K1(2)*Xcal + K1(3)*Ycal;
    plot3(Xcal,Ycal,Zplane,'.')
    
    legend('Nominal parameter','Calibrated parameters','Calibration plane')
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')
