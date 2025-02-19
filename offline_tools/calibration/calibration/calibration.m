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

%% INTRINSIC CALIBRATION
 % From flat ground data segmented_data.mat [4 segmented ground from 4 different positions]

 % Initial Parameters NOMINAL [From meassured/datasheet data]
    % angles in rads and discantes in meters
    alpha1= -1.2*pi/180;
    alpha2= -0.4*pi/180;
    alpha3= +0.4*pi/180;
    alpha4= +1.2*pi/180;
    ALPHA = [alpha1,alpha2,alpha3,alpha4];
    L1z   = (100.5786)/1000.0;
    L2z   = (8.3186)/100.0;
    L2y   = (18.8)/1000.0;
    L2x   = (-23.76)/1000.0;
    beta_off1=0;
    beta_off2=0;
    % Estimation vector W:
    W=[L1z,L2x,L2y,L2z,alpha1,alpha2,alpha3,alpha4,beta_off1,beta_off2];

% Data conditioning

    data = [data_ground_1; data_ground_2; data_ground_3; data_ground_4];
    
    echo=data(:,6);
    pos=find(echo==0);
    DATA=data(pos,:);
    scan=DATA(:,10);
    z=DATA(:,14);
    pos=find(z>-0.2 & z<0.15);
    DATA=DATA(pos,:);
    beta=DATA(:,3);
    pos=find(beta>-0.6 & beta<0.52);
    DATA=DATA(pos,:);

    layer=DATA(:,4);
    pos_1 = find(layer==0);
    pos_2 = find(layer==1);
    pos_3 = find(layer==2);
    pos_4 = find(layer==3);
    DATA_1=DATA(pos_1,:);
    DATA_2=DATA(pos_2,:);
    DATA_3=DATA(pos_3,:);
    DATA_4=DATA(pos_4,:);

    alpha=[alpha1*ones(size(DATA_1(:,1))); alpha2*ones(size(DATA_2(:,1))); alpha3*ones(size(DATA_3(:,1))); alpha4*ones(size(DATA_4(:,1))) ];
    r=[DATA_1(:,5);DATA_2(:,5);DATA_3(:,5);DATA_4(:,5)];
    tilt = [DATA_1(:,1);DATA_2(:,1);DATA_3(:,1);DATA_4(:,1)];
    beta= [DATA_1(:,3);DATA_2(:,3);DATA_3(:,3);DATA_4(:,3)];
    scan = [DATA_1(:,10);DATA_2(:,10);DATA_3(:,10);DATA_4(:,10)];
    pw = [DATA_1(:,7);DATA_2(:,7);DATA_3(:,7);DATA_4(:,7)];

    S=[length(DATA_1(:,1)),length(DATA_2(:,1)),length(DATA_3(:,1)),length(DATA_4(:,1))];

    % re-ordering of data
    
    alpha = [W(5)*ones(S(1),1);W(6)*ones(S(2),1);W(7)*ones(S(3),1);W(8)*ones(S(4),1)];
    beta_off =  [W(9)+zeros(S(1),1);W(9)+zeros(S(2),1); W(10)+zeros(S(3),1);W(10)+zeros(S(4),1)];
    X = W(2) - r.*cos(alpha).*sin(beta+beta_off);
    Y = W(3) - r.*sin(alpha).*sin(tilt) + r.*cos(alpha).*cos(beta+beta_off).*cos(tilt);
    Z = W(1) + W(4) + r.*sin(alpha).*cos(tilt) + r.*cos(alpha).*cos(beta+beta_off).*sin(tilt);
    Weight = 0.1*ones(size(Y));
    posy=find(X>-1.9 & X<1.5);
    Weight(posy)=1;
    posbeta_bad=find(beta==0.519235432148000);
    Weight(posbeta_bad)=0.0;
    wp_pos=find(Weight==1);
    wp_neg=find(Weight~=1);
    % Pulse width filtering (noisy points)
    posPW_bad=find(pw<1.44 & pw>2.2);
    Weight(posPW_bad)=0.1;
    
    figure(1)
        hold on
        plot3(X(wp_pos),Y(wp_pos),Z(wp_pos),'b.')
        hold on
        grid on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        plot3(X(wp_neg),Y(wp_neg),Z(wp_neg),'k.')
        legend('Data for estimation','Filtered data')
        grid on
        xlabel('X')
        ylabel('Y')


% OPTIMIZATION for intrinsic
    options = optimoptions(@lsqnonlin,'Display','iter');%'Algorithm','levenberg-marquardt'
    % lower bound
    lb=[0.02,-0.3,-0.1,0.02,-2.0*pi/180,-0.9*pi/180,0*pi/180,0.8*pi/180, -0.1, -0.1];
    % upper bound
    ub=[0.2,0.2,0.2,0.2,-0.75*pi/180,0*pi/180,0.75*pi/180,2*pi/180, 0.1, 0.1];
    % Initial plane
    M=[ones(size(X)), X, Y];
    K1=lscov(M,Z,Weight);
    % Estimation function
    f= @(W)model_01(W, K1, r, beta, tilt, S, Weight );
    tic
    [W2,resnorm,residual,exitflag,output] = lsqnonlin(f,W,lb,ub,options);
    toc
    L1z=W2(1);L2x=W2(2);L2y=W2(3);L2z=W2(4);alpha1=W2(5);alpha2=W2(6);alpha3=W2(7);alpha4=W2(8); beta_off1=W2(9); beta_off2=W2(10);
    disp('New vector parameters for intrinsic:')
    disp(W2)
    
    % plotting adjusted data by layers
    alpha    = alpha1;
    r      = DATA_1(:,5);
    tilt   = DATA_1(:,1);
    betas  =  DATA_1(:,3);
    beta_off = beta_off1;
    X1 = L2x - r.*cos(alpha).*sin(betas+beta_off);
    Y1 = L2y - r.*sin(alpha).*sin(tilt) + r.*cos(alpha).*cos(betas+beta_off).*cos(tilt);
    Z1 = L1z + L2z + r.*sin(alpha).*cos(tilt) + r.*cos(alpha).*cos(betas+beta_off).*sin(tilt);
    
    alpha    = alpha2;
    r      = DATA_2(:,5);
    tilt   = DATA_2(:,1);
    betas  =  DATA_2(:,3);
    beta_off = beta_off2;
    X2 = L2x - r.*cos(alpha).*sin(betas+beta_off);
    Y2 = L2y - r.*sin(alpha).*sin(tilt) + r.*cos(alpha).*cos(betas+beta_off).*cos(tilt);
    Z2 = L1z + L2z + r.*sin(alpha).*cos(tilt) + r.*cos(alpha).*cos(betas+beta_off).*sin(tilt);
    
    
    alpha    = alpha3;
    r      = DATA_3(:,5);
    tilt   = DATA_3(:,1);
    betas  =  DATA_3(:,3);
    beta_off = beta_off2;
    X3 = L2x - r.*cos(alpha).*sin(betas+beta_off);
    Y3 = L2y - r.*sin(alpha).*sin(tilt) + r.*cos(alpha).*cos(betas+beta_off).*cos(tilt);
    Z3 = L1z + L2z + r.*sin(alpha).*cos(tilt) + r.*cos(alpha).*cos(betas+beta_off).*sin(tilt);
    
    
    alpha    = alpha4;
    r      = DATA_4(:,5);
    tilt   = DATA_4(:,1);
    betas  =  DATA_4(:,3);
    beta_off = beta_off2;
    X4 = L2x - r.*cos(alpha).*sin(betas+beta_off);
    Y4 = L2y - r.*sin(alpha).*sin(tilt) + r.*cos(alpha).*cos(betas+beta_off).*cos(tilt);
    Z4 = L1z + L2z + r.*sin(alpha).*cos(tilt) + r.*cos(alpha).*cos(betas+beta_off).*sin(tilt);
    
    figure(2)
        plot3(X,Y,Z,'.')
        hold on
        grid on
        plot3(X1,Y1,Z1,'.')
        plot3(X2,Y2,Z2,'.')
        plot3(X3,Y3,Z3,'.')
        plot3(X4,Y4,Z4,'.')
        xlabel('X')
        xlabel('Y')
        xlabel('Z')
        legend('Nominal parameters','Adjusted L1', 'Adjusted L2', 'Adjusted L3', 'Adjusted L4')
        
     
%% EXTRINSEC CALIBRATION  
 % From wall data segmented_data.mat [4 segmented walls from 4 different positions]

 % Initial Parameters NOMINAL [From meassured/datasheet info]
    % angles in rads and discantes in meters

    % Initial Extrinsic
    % angles in radas, distances in meters
    Loz      = 76.0/100.0;
    Loy      = 46.0/100.0;
    Lox      = -9.24/1000.0;
    pitch_2  = 0;
    yaw      = 0;
    pitch    = 0;
    roll     = 0;
    tilt_off = 0;
    % Estimation vector:
    W=[Lox,Loy,Loz,pitch,roll,yaw];

 % Optimization for extrinsic
    
    options = optimoptions(@lsqnonlin,'Display','iter','MaxFunctionEvaluations', 10000);
    fE= @(W)model_E1( W, data_surf_1, data_surf_2, data_surf_3, data_surf_4, data_ground_1, data_ground_2, data_ground_3, data_ground_4 );
    tic
    [W2,resnorm,residual,exitflag,output] = lsqnonlin(fE,W,[],[],options);
    toc
    disp('New vector parameters for extrinsic:')
    disp(W2)

 % FINAL Extrinsic Evaluation
     Lox=W2(1); Loy=W2(2); Loz=W2(3); pitch=W2(4); roll=W2(5); yaw=W2(6);
     W = W2;
     
    layer_s1 = [data_surf_1(:,4); data_ground_1(:,4)];
    layer_s2 = [data_surf_2(:,4); data_ground_2(:,4)];
    layer_s3 = [data_surf_3(:,4); data_ground_3(:,4)];
    layer_s4 = [data_surf_4(:,4); data_ground_4(:,4)];
    ALPHA_1=zeros(size(layer_s1));
    ALPHA_2=zeros(size(layer_s2));
    ALPHA_3=zeros(size(layer_s3));
    ALPHA_4=zeros(size(layer_s4));
    a1_s1=find(layer_s1==0);
    a2_s1=find(layer_s1==1);
    a3_s1=find(layer_s1==2);
    a4_s1=find(layer_s1==3);
    a1_s2=find(layer_s2==0);
    a2_s2=find(layer_s2==1);
    a3_s2=find(layer_s2==2);
    a4_s2=find(layer_s2==3);
    a1_s3=find(layer_s3==0);
    a2_s3=find(layer_s3==1);
    a3_s3=find(layer_s3==2);
    a4_s3=find(layer_s3==3);
    a1_s4=find(layer_s4==0);
    a2_s4=find(layer_s4==1);
    a3_s4=find(layer_s4==2);
    a4_s4=find(layer_s4==3);
    
    ALPHA_1(a1_s1)=alpha1; ALPHA_1(a2_s1)=alpha2; ALPHA_1(a3_s1)=alpha3; ALPHA_1(a4_s1)=alpha4;
    ALPHA_2(a1_s2)=alpha1; ALPHA_2(a2_s2)=alpha2; ALPHA_2(a3_s2)=alpha3; ALPHA_2(a4_s2)=alpha4;
    ALPHA_3(a1_s3)=alpha1; ALPHA_3(a2_s3)=alpha2; ALPHA_3(a3_s3)=alpha3; ALPHA_3(a4_s3)=alpha4;
    ALPHA_4(a1_s4)=alpha1; ALPHA_4(a2_s4)=alpha2; ALPHA_4(a3_s4)=alpha3; ALPHA_4(a4_s4)=alpha4;
    
    % Adding position of rover: from minnie pose topic
    Minnie_Q_wxyz = [0.999999999274, 0, 0, -3.81075057872e-05; ...
                     0.967270826375, 0, 0, 0.253746228432;...
                     0.968700982599, 0, 0, -0.248230550721; ...
                     0.9999127301, 0, 0, .0132110629169];

    [Minnie_pitch, Minnie_roll, Minnie_yaw] = quat2angle(Minnie_Q_wxyz(1:4,:),'YXZ');
    Minnie_yaw= -Minnie_yaw;
    
    Minnie_POS_XYZ=[-0.0011072741878 8.84206411503e-08 0; ...
                4.04728161632, -1.25454522771, 0; ...
                5.49540472942, 1.19624652699, 0;
                5.04887363497, -0.271461711511, 0];
    
    aux = Minnie_POS_XYZ(:,1);
    Minnie_POS_XYZ(:,1) = -Minnie_POS_XYZ(:,2);
    Minnie_POS_XYZ(:,2) = aux;

    alpha_1 = ALPHA_1; alpha_2 = ALPHA_2; alpha_3 = ALPHA_3; alpha_4 = ALPHA_4;
    beta_1  = [data_surf_1(:,3)+beta_off1; data_ground_1(:,3)+beta_off1 ]; 
    beta_2  = [data_surf_2(:,3)+beta_off1; data_ground_2(:,3)+beta_off1];
    beta_3  = [data_surf_3(:,3)+beta_off2; data_ground_3(:,3)+beta_off2];
    beta_4  = [data_surf_4(:,3)+beta_off2; data_ground_4(:,3)+beta_off2];
    tilt_1  = [data_surf_1(:,1); data_ground_1(:,1)]; 
    tilt_2  = [data_surf_2(:,1); data_ground_2(:,1)]; 
    tilt_3  = [data_surf_3(:,1); data_ground_3(:,1)];  
    tilt_4  = [data_surf_4(:,1); data_ground_4(:,1)]; 
    r_1     = [data_surf_1(:,5); data_ground_1(:,5)]; 
    r_2     = [data_surf_2(:,5); data_ground_2(:,5)]; 
    r_3     = [data_surf_3(:,5); data_ground_3(:,5)]; 
    r_4     = [data_surf_4(:,5); data_ground_4(:,5)]; 
    
    % spliting by position of rover
        beta_1  = [data1(:,3)+beta_off1]; 
        beta_2  = [data2(:,3)+beta_off1];
        beta_3  = [data3(:,3)+beta_off2];
        beta_4  = [data4(:,3)+beta_off2];
        tilt_1  = [data1(:,1)]; 
        tilt_2  = [data2(:,1)]; 
        tilt_3  = [data3(:,1)]; 
        tilt_4  = [data4(:,1)]; 
        r_1     = [data1(:,5)]; 
        r_2     = [data2(:,5)]; 
        r_3     = [data3(:,5)]; 
        r_4     = [data4(:,5)]; 

        layer_s1 = [data1(:,4)];
        layer_s2 = [data2(:,4)];
        layer_s3 = [data3(:,4)];
        layer_s4 = [data4(:,4)];
        
        ALPHA_1=zeros(size(layer_s1));
        ALPHA_2=zeros(size(layer_s2));
        ALPHA_3=zeros(size(layer_s3));
        ALPHA_4=zeros(size(layer_s4));
        a1_s1=find(layer_s1==0);
        a2_s1=find(layer_s1==1);
        a3_s1=find(layer_s1==2);
        a4_s1=find(layer_s1==3);
        a1_s2=find(layer_s2==0);
        a2_s2=find(layer_s2==1);
        a3_s2=find(layer_s2==2);
        a4_s2=find(layer_s2==3);
        a1_s3=find(layer_s3==0);
        a2_s3=find(layer_s3==1);
        a3_s3=find(layer_s3==2);
        a4_s3=find(layer_s3==3);
        a1_s4=find(layer_s4==0);
        a2_s4=find(layer_s4==1);
        a3_s4=find(layer_s4==2);
        a4_s4=find(layer_s4==3);

        ALPHA_1(a1_s1)=alpha1; ALPHA_1(a2_s1)=alpha2; ALPHA_1(a3_s1)=alpha3; ALPHA_1(a4_s1)=alpha4;
        ALPHA_2(a1_s2)=alpha1; ALPHA_2(a2_s2)=alpha2; ALPHA_2(a3_s2)=alpha3; ALPHA_2(a4_s2)=alpha4;
        ALPHA_3(a1_s3)=alpha1; ALPHA_3(a2_s3)=alpha2; ALPHA_3(a3_s3)=alpha3; ALPHA_3(a4_s3)=alpha4;
        ALPHA_4(a1_s4)=alpha1; ALPHA_4(a2_s4)=alpha2; ALPHA_4(a3_s4)=alpha3; ALPHA_4(a4_s4)=alpha4;
        alpha_1 = ALPHA_1; alpha_2 = ALPHA_2; alpha_3 = ALPHA_3; alpha_4 = ALPHA_4;
    
    ang_1   = -Minnie_yaw(1)*ones(size(layer_s1)); ang_2 = -Minnie_yaw(2)*ones(size(layer_s2));
    ang_3   = -Minnie_yaw(3)*ones(size(layer_s3)); ang_4 = -Minnie_yaw(4)*ones(size(layer_s4));
    off_x_1 = Minnie_POS_XYZ(1,1)*ones(size(layer_s1)); off_x_2 = Minnie_POS_XYZ(2,1)*ones(size(layer_s2));
    off_x_3 = Minnie_POS_XYZ(3,1)*ones(size(layer_s3)); off_x_4 = Minnie_POS_XYZ(4,1)*ones(size(layer_s4));
    off_y_1 = Minnie_POS_XYZ(1,2)*ones(size(layer_s1)); off_y_2 = Minnie_POS_XYZ(2,2)*ones(size(layer_s2));
    off_y_3 = Minnie_POS_XYZ(3,2)*ones(size(layer_s3)); off_y_4 = Minnie_POS_XYZ(4,2)*ones(size(layer_s4));
    
    % Calc the X_1, Y_1, Z_1 for position 1
   
    X_1 = off_x_1 + L2x.*(cos(W(5)).*(cos(ang_1).*cos(W(6)) - sin(ang_1).*sin(W(6))) - ...
        sin(W(4)).*sin(W(5)).*(cos(ang_1).*sin(W(6)) + sin(ang_1).*cos(W(6)))) + ...
        L1z.*(sin(W(5)).*(cos(ang_1).*cos(W(6)) - sin(ang_1).*sin(W(6))) + ...
        cos(W(5)).*sin(W(4)).*(cos(ang_1).*sin(W(6)) + sin(ang_1).*cos(W(6)))) + ...
        L2z.*(sin(W(5)).*(cos(ang_1).*cos(W(6)) - sin(ang_1).*sin(W(6))) + ...
        cos(W(5)).*sin(W(4)).*(cos(ang_1).*sin(W(6)) + sin(ang_1).*cos(W(6)))) + ...
        W(1).*cos(ang_1) - W(2).*sin(ang_1) - L2y.*cos(W(4)).*(cos(ang_1).*sin(W(6)) + ...
        sin(ang_1).*cos(W(6))) + r_1.*sin(alpha_1).*(cos(tilt_1).*(sin(W(5)).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6))) + cos(W(5)).*sin(W(4)).*(cos(ang_1).*sin(W(6)) + ...
        sin(ang_1).*cos(W(6)))) + cos(W(4)).*sin(tilt_1).*(cos(ang_1).*sin(W(6)) + ...
        sin(ang_1).*cos(W(6)))) - r_1.*cos(alpha_1).*sin(beta_1).*(cos(W(5)).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6))) - sin(W(4)).*sin(W(5)).*(cos(ang_1).*sin(W(6)) + ...
        sin(ang_1).*cos(W(6)))) + r_1.*cos(alpha_1).*cos(beta_1).*(sin(tilt_1).*(sin(W(5)).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6))) + cos(W(5)).*sin(W(4)).*(cos(ang_1).*sin(W(6)) + sin(ang_1).*cos(W(6)))) - ...
        cos(W(4)).*cos(tilt_1).*(cos(ang_1).*sin(W(6)) + sin(ang_1).*cos(W(6))));
 
    Y_1 = off_y_1 + L2x.*(cos(W(5)).*(cos(ang_1).*sin(W(6)) + sin(ang_1).*cos(W(6))) + ...
        sin(W(4)).*sin(W(5)).*(cos(ang_1).*cos(W(6)) - sin(ang_1).*sin(W(6)))) + ...
        L1z.*(sin(W(5)).*(cos(ang_1).*sin(W(6)) + sin(ang_1).*cos(W(6))) - ...
        cos(W(5)).*sin(W(4)).*(cos(ang_1).*cos(W(6)) - sin(ang_1).*sin(W(6)))) + ...
        L2z.*(sin(W(5)).*(cos(ang_1).*sin(W(6)) + sin(ang_1).*cos(W(6))) - ...
        cos(W(5)).*sin(W(4)).*(cos(ang_1).*cos(W(6)) - sin(ang_1).*sin(W(6)))) + ...
        W(2).*cos(ang_1) + W(1).*sin(ang_1) + L2y.*cos(W(4)).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6))) + r_1.*sin(alpha_1).*(cos(tilt_1).*(sin(W(5)).*(cos(ang_1).*sin(W(6)) + ...
        sin(ang_1).*cos(W(6))) - cos(W(5)).*sin(W(4)).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6)))) - cos(W(4)).*sin(tilt_1).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6)))) - r_1.*cos(alpha_1).*sin(beta_1).*(cos(W(5)).*(cos(ang_1).*sin(W(6)) + ...
        sin(ang_1).*cos(W(6))) + sin(W(4)).*sin(W(5)).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6)))) + r_1.*cos(alpha_1).*cos(beta_1).*(sin(tilt_1).*(sin(W(5)).*(cos(ang_1).*sin(W(6)) + ...
        sin(ang_1).*cos(W(6))) - cos(W(5)).*sin(W(4)).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6)))) + cos(W(4)).*cos(tilt_1).*(cos(ang_1).*cos(W(6)) - ...
        sin(ang_1).*sin(W(6))));
    
    Z_1 = W(3) + L2y.*sin(W(4)) - r_1.*sin(alpha_1).*(sin(W(4)).*sin(tilt_1) - ...
        cos(W(4)).*cos(W(5)).*cos(tilt_1)) + L1z.*cos(W(4)).*cos(W(5)) + ...
        L2z.*cos(W(4)).*cos(W(5)) - L2x.*cos(W(4)).*sin(W(5)) + ...
        r_1.*cos(alpha_1).*cos(beta_1).*(cos(tilt_1).*sin(W(4)) + cos(W(4)).*cos(W(5)).*sin(tilt_1)) + ...
        r_1.*cos(alpha_1).*cos(W(4)).*sin(beta_1).*sin(W(5));
    
    % Calc the X_2, Y_2, Z_2 for position 2
    
    X_2 = off_x_2 + L2x.*(cos(W(5)).*(cos(ang_2).*cos(W(6)) - sin(ang_2).*sin(W(6))) - ...
        sin(W(4)).*sin(W(5)).*(cos(ang_2).*sin(W(6)) + sin(ang_2).*cos(W(6)))) + ...
        L1z.*(sin(W(5)).*(cos(ang_2).*cos(W(6)) - sin(ang_2).*sin(W(6))) + ...
        cos(W(5)).*sin(W(4)).*(cos(ang_2).*sin(W(6)) + sin(ang_2).*cos(W(6)))) + ...
        L2z.*(sin(W(5)).*(cos(ang_2).*cos(W(6)) - sin(ang_2).*sin(W(6))) + ...
        cos(W(5)).*sin(W(4)).*(cos(ang_2).*sin(W(6)) + sin(ang_2).*cos(W(6)))) + ...
        W(1).*cos(ang_2) - W(2).*sin(ang_2) - L2y.*cos(W(4)).*(cos(ang_2).*sin(W(6)) + ...
        sin(ang_2).*cos(W(6))) + r_2.*sin(alpha_2).*(cos(tilt_2).*(sin(W(5)).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6))) + cos(W(5)).*sin(W(4)).*(cos(ang_2).*sin(W(6)) + ...
        sin(ang_2).*cos(W(6)))) + cos(W(4)).*sin(tilt_2).*(cos(ang_2).*sin(W(6)) + ...
        sin(ang_2).*cos(W(6)))) - r_2.*cos(alpha_2).*sin(beta_2).*(cos(W(5)).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6))) - sin(W(4)).*sin(W(5)).*(cos(ang_2).*sin(W(6)) + ...
        sin(ang_2).*cos(W(6)))) + r_2.*cos(alpha_2).*cos(beta_2).*(sin(tilt_2).*(sin(W(5)).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6))) + cos(W(5)).*sin(W(4)).*(cos(ang_2).*sin(W(6)) + sin(ang_2).*cos(W(6)))) - ...
        cos(W(4)).*cos(tilt_2).*(cos(ang_2).*sin(W(6)) + sin(ang_2).*cos(W(6))));
 
    Y_2 = off_y_2 + L2x.*(cos(W(5)).*(cos(ang_2).*sin(W(6)) + sin(ang_2).*cos(W(6))) + ...
        sin(W(4)).*sin(W(5)).*(cos(ang_2).*cos(W(6)) - sin(ang_2).*sin(W(6)))) + ...
        L1z.*(sin(W(5)).*(cos(ang_2).*sin(W(6)) + sin(ang_2).*cos(W(6))) - ...
        cos(W(5)).*sin(W(4)).*(cos(ang_2).*cos(W(6)) - sin(ang_2).*sin(W(6)))) + ...
        L2z.*(sin(W(5)).*(cos(ang_2).*sin(W(6)) + sin(ang_2).*cos(W(6))) - ...
        cos(W(5)).*sin(W(4)).*(cos(ang_2).*cos(W(6)) - sin(ang_2).*sin(W(6)))) + ...
        W(2).*cos(ang_2) + W(1).*sin(ang_2) + L2y.*cos(W(4)).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6))) + r_2.*sin(alpha_2).*(cos(tilt_2).*(sin(W(5)).*(cos(ang_2).*sin(W(6)) + ...
        sin(ang_2).*cos(W(6))) - cos(W(5)).*sin(W(4)).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6)))) - cos(W(4)).*sin(tilt_2).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6)))) - r_2.*cos(alpha_2).*sin(beta_2).*(cos(W(5)).*(cos(ang_2).*sin(W(6)) + ...
        sin(ang_2).*cos(W(6))) + sin(W(4)).*sin(W(5)).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6)))) + r_2.*cos(alpha_2).*cos(beta_2).*(sin(tilt_2).*(sin(W(5)).*(cos(ang_2).*sin(W(6)) + ...
        sin(ang_2).*cos(W(6))) - cos(W(5)).*sin(W(4)).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6)))) + cos(W(4)).*cos(tilt_2).*(cos(ang_2).*cos(W(6)) - ...
        sin(ang_2).*sin(W(6))));
    
    Z_2 = W(3) + L2y.*sin(W(4)) - r_2.*sin(alpha_2).*(sin(W(4)).*sin(tilt_2) - ...
        cos(W(4)).*cos(W(5)).*cos(tilt_2)) + L1z.*cos(W(4)).*cos(W(5)) + ...
        L2z.*cos(W(4)).*cos(W(5)) - L2x.*cos(W(4)).*sin(W(5)) + ...
        r_2.*cos(alpha_2).*cos(beta_2).*(cos(tilt_2).*sin(W(4)) + cos(W(4)).*cos(W(5)).*sin(tilt_2)) + ...
        r_2.*cos(alpha_2).*cos(W(4)).*sin(beta_2).*sin(W(5));
    
    % Calc the X_3, Y_3, Z_3 for position 3
    
    X_3 = off_x_3 + L2x.*(cos(W(5)).*(cos(ang_3).*cos(W(6)) - sin(ang_3).*sin(W(6))) - ...
        sin(W(4)).*sin(W(5)).*(cos(ang_3).*sin(W(6)) + sin(ang_3).*cos(W(6)))) + ...
        L1z.*(sin(W(5)).*(cos(ang_3).*cos(W(6)) - sin(ang_3).*sin(W(6))) + ...
        cos(W(5)).*sin(W(4)).*(cos(ang_3).*sin(W(6)) + sin(ang_3).*cos(W(6)))) + ...
        L2z.*(sin(W(5)).*(cos(ang_3).*cos(W(6)) - sin(ang_3).*sin(W(6))) + ...
        cos(W(5)).*sin(W(4)).*(cos(ang_3).*sin(W(6)) + sin(ang_3).*cos(W(6)))) + ...
        W(1).*cos(ang_3) - W(2).*sin(ang_3) - L2y.*cos(W(4)).*(cos(ang_3).*sin(W(6)) + ...
        sin(ang_3).*cos(W(6))) + r_3.*sin(alpha_3).*(cos(tilt_3).*(sin(W(5)).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6))) + cos(W(5)).*sin(W(4)).*(cos(ang_3).*sin(W(6)) + ...
        sin(ang_3).*cos(W(6)))) + cos(W(4)).*sin(tilt_3).*(cos(ang_3).*sin(W(6)) + ...
        sin(ang_3).*cos(W(6)))) - r_3.*cos(alpha_3).*sin(beta_3).*(cos(W(5)).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6))) - sin(W(4)).*sin(W(5)).*(cos(ang_3).*sin(W(6)) + ...
        sin(ang_3).*cos(W(6)))) + r_3.*cos(alpha_3).*cos(beta_3).*(sin(tilt_3).*(sin(W(5)).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6))) + cos(W(5)).*sin(W(4)).*(cos(ang_3).*sin(W(6)) + sin(ang_3).*cos(W(6)))) - ...
        cos(W(4)).*cos(tilt_3).*(cos(ang_3).*sin(W(6)) + sin(ang_3).*cos(W(6))));
 
    Y_3 = off_y_3 + L2x.*(cos(W(5)).*(cos(ang_3).*sin(W(6)) + sin(ang_3).*cos(W(6))) + ...
        sin(W(4)).*sin(W(5)).*(cos(ang_3).*cos(W(6)) - sin(ang_3).*sin(W(6)))) + ...
        L1z.*(sin(W(5)).*(cos(ang_3).*sin(W(6)) + sin(ang_3).*cos(W(6))) - ...
        cos(W(5)).*sin(W(4)).*(cos(ang_3).*cos(W(6)) - sin(ang_3).*sin(W(6)))) + ...
        L2z.*(sin(W(5)).*(cos(ang_3).*sin(W(6)) + sin(ang_3).*cos(W(6))) - ...
        cos(W(5)).*sin(W(4)).*(cos(ang_3).*cos(W(6)) - sin(ang_3).*sin(W(6)))) + ...
        W(2).*cos(ang_3) + W(1).*sin(ang_3) + L2y.*cos(W(4)).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6))) + r_3.*sin(alpha_3).*(cos(tilt_3).*(sin(W(5)).*(cos(ang_3).*sin(W(6)) + ...
        sin(ang_3).*cos(W(6))) - cos(W(5)).*sin(W(4)).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6)))) - cos(W(4)).*sin(tilt_3).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6)))) - r_3.*cos(alpha_3).*sin(beta_3).*(cos(W(5)).*(cos(ang_3).*sin(W(6)) + ...
        sin(ang_3).*cos(W(6))) + sin(W(4)).*sin(W(5)).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6)))) + r_3.*cos(alpha_3).*cos(beta_3).*(sin(tilt_3).*(sin(W(5)).*(cos(ang_3).*sin(W(6)) + ...
        sin(ang_3).*cos(W(6))) - cos(W(5)).*sin(W(4)).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6)))) + cos(W(4)).*cos(tilt_3).*(cos(ang_3).*cos(W(6)) - ...
        sin(ang_3).*sin(W(6))));
    
    Z_3 = W(3) + L2y.*sin(W(4)) - r_3.*sin(alpha_3).*(sin(W(4)).*sin(tilt_3) - ...
        cos(W(4)).*cos(W(5)).*cos(tilt_3)) + L1z.*cos(W(4)).*cos(W(5)) + ...
        L2z.*cos(W(4)).*cos(W(5)) - L2x.*cos(W(4)).*sin(W(5)) + ...
        r_3.*cos(alpha_3).*cos(beta_3).*(cos(tilt_3).*sin(W(4)) + cos(W(4)).*cos(W(5)).*sin(tilt_3)) + ...
        r_3.*cos(alpha_3).*cos(W(4)).*sin(beta_3).*sin(W(5));
    
    % Calc the X_4, Y_4, Z_4 for position 4
    
    X_4 = off_x_4 + L2x.*(cos(W(5)).*(cos(ang_4).*cos(W(6)) - sin(ang_4).*sin(W(6))) - ...
        sin(W(4)).*sin(W(5)).*(cos(ang_4).*sin(W(6)) + sin(ang_4).*cos(W(6)))) + ...
        L1z.*(sin(W(5)).*(cos(ang_4).*cos(W(6)) - sin(ang_4).*sin(W(6))) + ...
        cos(W(5)).*sin(W(4)).*(cos(ang_4).*sin(W(6)) + sin(ang_4).*cos(W(6)))) + ...
        L2z.*(sin(W(5)).*(cos(ang_4).*cos(W(6)) - sin(ang_4).*sin(W(6))) + ...
        cos(W(5)).*sin(W(4)).*(cos(ang_4).*sin(W(6)) + sin(ang_4).*cos(W(6)))) + ...
        W(1).*cos(ang_4) - W(2).*sin(ang_4) - L2y.*cos(W(4)).*(cos(ang_4).*sin(W(6)) + ...
        sin(ang_4).*cos(W(6))) + r_4.*sin(alpha_4).*(cos(tilt_4).*(sin(W(5)).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6))) + cos(W(5)).*sin(W(4)).*(cos(ang_4).*sin(W(6)) + ...
        sin(ang_4).*cos(W(6)))) + cos(W(4)).*sin(tilt_4).*(cos(ang_4).*sin(W(6)) + ...
        sin(ang_4).*cos(W(6)))) - r_4.*cos(alpha_4).*sin(beta_4).*(cos(W(5)).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6))) - sin(W(4)).*sin(W(5)).*(cos(ang_4).*sin(W(6)) + ...
        sin(ang_4).*cos(W(6)))) + r_4.*cos(alpha_4).*cos(beta_4).*(sin(tilt_4).*(sin(W(5)).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6))) + cos(W(5)).*sin(W(4)).*(cos(ang_4).*sin(W(6)) + sin(ang_4).*cos(W(6)))) - ...
        cos(W(4)).*cos(tilt_4).*(cos(ang_4).*sin(W(6)) + sin(ang_4).*cos(W(6))));
 
    Y_4 = off_y_4 + L2x.*(cos(W(5)).*(cos(ang_4).*sin(W(6)) + sin(ang_4).*cos(W(6))) + ...
        sin(W(4)).*sin(W(5)).*(cos(ang_4).*cos(W(6)) - sin(ang_4).*sin(W(6)))) + ...
        L1z.*(sin(W(5)).*(cos(ang_4).*sin(W(6)) + sin(ang_4).*cos(W(6))) - ...
        cos(W(5)).*sin(W(4)).*(cos(ang_4).*cos(W(6)) - sin(ang_4).*sin(W(6)))) + ...
        L2z.*(sin(W(5)).*(cos(ang_4).*sin(W(6)) + sin(ang_4).*cos(W(6))) - ...
        cos(W(5)).*sin(W(4)).*(cos(ang_4).*cos(W(6)) - sin(ang_4).*sin(W(6)))) + ...
        W(2).*cos(ang_4) + W(1).*sin(ang_4) + L2y.*cos(W(4)).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6))) + r_4.*sin(alpha_4).*(cos(tilt_4).*(sin(W(5)).*(cos(ang_4).*sin(W(6)) + ...
        sin(ang_4).*cos(W(6))) - cos(W(5)).*sin(W(4)).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6)))) - cos(W(4)).*sin(tilt_4).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6)))) - r_4.*cos(alpha_4).*sin(beta_4).*(cos(W(5)).*(cos(ang_4).*sin(W(6)) + ...
        sin(ang_4).*cos(W(6))) + sin(W(4)).*sin(W(5)).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6)))) + r_4.*cos(alpha_4).*cos(beta_4).*(sin(tilt_4).*(sin(W(5)).*(cos(ang_4).*sin(W(6)) + ...
        sin(ang_4).*cos(W(6))) - cos(W(5)).*sin(W(4)).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6)))) + cos(W(4)).*cos(tilt_4).*(cos(ang_4).*cos(W(6)) - ...
        sin(ang_4).*sin(W(6))));
    
    Z_4 = W(3) + L2y.*sin(W(4)) - r_4.*sin(alpha_4).*(sin(W(4)).*sin(tilt_4) - ...
        cos(W(4)).*cos(W(5)).*cos(tilt_4)) + L1z.*cos(W(4)).*cos(W(5)) + ...
        L2z.*cos(W(4)).*cos(W(5)) - L2x.*cos(W(4)).*sin(W(5)) + ...
        r_4.*cos(alpha_4).*cos(beta_4).*(cos(tilt_4).*sin(W(4)) + cos(W(4)).*cos(W(5)).*sin(tilt_4)) + ...
        r_4.*cos(alpha_4).*cos(W(4)).*sin(beta_4).*sin(W(5));
    
    % plotting the data
    plot3(X_1,Y_1,Z_1,'.')
    hold on
    grid on
    plot3(X_2,Y_2,Z_2,'.')
    plot3(X_3,Y_3,Z_3,'.')
    plot3(X_4,Y_4,Z_4,'.')
    axis([-1 2.5 8.9 9.3 0.2 2])
    legend('Adjuste wall-cloud from position 1', 'Adjuste wall-cloud  from position 2', 'Adjuste wall-cloud  from position 3','Adjuste wall-cloud  from position 4')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    
    