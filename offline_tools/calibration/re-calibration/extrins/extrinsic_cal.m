%% Extrinsic re-calibration
%  calibration of intrinsic parameters by using a street scanning on file
%  cement_03.txt previusly segmented

% LAAS-CNRS: Robotic and Interaction Systems
% SICK LDMRS, Platine Light
% Harold F. MURCIA - November 2017

clear, clc, close all

%% Initial Parameters intrisic
    L1z         =   0.072236184455477;
    L2x         =   0.100587885615162;
    L2y         =  -0.074991685813609;
    L2z         =   0.064653815212933;
    alpha1     =  -0.031272870583227;
    alpha2     =  -0.012131387660205;
    alpha3     =   0.005235987756497;
    alpha4     =   0.019336219601170;
    ALPHA = [alpha1,alpha2,alpha3,alpha4];
    beta_off1   =  -0.011554163109727;
    beta_off2   =  -0.016255065762611;
    tilt_off    =  7.532319467863496e-04;
    beta_off = 0;

    L1z         =  0.100578984193110;
    L2x         = -0.023760000000000;
    L2y         =  0.018797850189242;
    L2z         =  0.083186331806302;
    alpha1     =  -0.026310185433179;
    alpha2     =  -0.007321821152752;
    alpha3     =   0.008640101362527;
    alpha4     =   0.023020415513343;
    ALPHA = [alpha1,alpha2,alpha3,alpha4];
    beta_off1   =  1.049615948354090e-10;
    beta_off2   =  -1.244528810882754e-11;
    tilt_off    =   -3.962113176508846e-05;
    beta_off = 0;

% NOMINLA EXT:
    roll=0;
    pitch =0;
    Loz= 0.75;
    W=[pitch,roll, Loz]; lb=[-0.1, -0.1, 0.5]; ub=[0.1,0.1, 1.3];
    %W=[L1z,L2x,L2y,L2z,alpha1,alpha2,alpha3,alpha4,beta_off1,beta_off2, tilt_off,pitch,roll, Loz];
    %lb=[0.05, -0.3, -0.15, 0.1, -2.0*pi/180,-0.8*pi/180,0.3*pi/180,1.1*pi/180, -0.1, -0.1, -2*0.0175,-0.1, -0.1, 0.5];
    %ub=[0.15, 0.2,0.2, 0.15, -1.1*pi/180, -0.3*pi/180, 0.8*pi/180, 1.6*pi/180, 0.1, 0.1, 2*0.0175 ,0.1,0.1, 1.3];
    
    
%% loading data from laas_street
    %data = load('/Users/haroldfmurcia/Desktop/re-calibration/extrins/cement_03.txt');
    data = load('/Users/haroldfmurcia/Desktop/re-calibration/intrins/laas_street.txt');
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
    
    X = L2x*cos(roll) + L1z*sin(roll) + L2z*sin(roll) + r.*cos(tilt + tilt_off).*sin(alpha).*sin(roll) - r.*cos(alpha).*sin(beta).*cos(roll) + r.*sin(tilt + tilt_off).*cos(alpha).*cos(beta).*sin(roll);
    Y = L2y*cos(pitch) - L1z*cos(roll)*sin(pitch) - L2z*cos(roll)*sin(pitch) + L2x*sin(pitch)*sin(roll) - r.*sin(alpha).*(sin(tilt + tilt_off).*cos(pitch) + cos(tilt + tilt_off).*cos(roll).*sin(pitch)) + r.*cos(alpha).*cos(beta).*(cos(tilt + tilt_off).*cos(pitch) - sin(tilt + tilt_off).*cos(roll).*sin(pitch)) - r.*cos(alpha).*sin(beta).*sin(pitch).*sin(roll);
    Z = Loz + L2y*sin(pitch) + L1z*cos(pitch)*cos(roll) + L2z*cos(pitch)*cos(roll) - L2x*cos(pitch)*sin(roll) - r.*sin(alpha).*(sin(tilt + tilt_off).*sin(pitch) - cos(tilt + tilt_off).*cos(pitch).*cos(roll)) + r.*cos(alpha).*cos(beta).*(cos(tilt + tilt_off).*sin(pitch) + sin(tilt + tilt_off).*cos(pitch)*cos(roll)) + r.*cos(alpha).*cos(pitch).*sin(beta).*sin(roll);
    
%% Filtering data by pulse widht
%         pw = data(:,7);
%         plot3(X,Y,pw,'.')
%         grid on
%         hold on
%         pw_u = mean(pw);
%         a = 0.5;
%         P=find(pw>(pw_u-a) & pw<(pw_u+a));
%         plot3(X(P),Y(P),pw(P),'.')
%         view(-90,0)
%         
%         DATA = data(P,:);
%         echo=DATA(:,6);
%         layer = DATA(:,4);
%         pos_1 = find(layer==0);
%         pos_2 = find(layer==1);
%         pos_3 = find(layer==2);
%         pos_4 = find(layer==3);
%         DATA_1=DATA(pos_1,:);
%         DATA_2=DATA(pos_2,:);
%         DATA_3=DATA(pos_3,:);
%         DATA_4=DATA(pos_4,:);
%         
%         alpha=[alpha1*ones(size(DATA_1(:,1))); alpha2*ones(size(DATA_2(:,1))); alpha3*ones(size(DATA_3(:,1))); alpha4*ones(size(DATA_4(:,1))) ];
%         r=[DATA_1(:,5);DATA_2(:,5);DATA_3(:,5);DATA_4(:,5)];
%         tilt = [DATA_1(:,1);DATA_2(:,1);DATA_3(:,1);DATA_4(:,1)];
%         beta= [DATA_1(:,3);DATA_2(:,3);DATA_3(:,3);DATA_4(:,3)];
%         scan = [DATA_1(:,10);DATA_2(:,10);DATA_3(:,10);DATA_4(:,10)];
%         pw = [DATA_1(:,7);DATA_2(:,7);DATA_3(:,7);DATA_4(:,7)];
%         S=[length(DATA_1(:,1)),length(DATA_2(:,1)),length(DATA_3(:,1)),length(DATA_4(:,1))];
%         tilt = tilt*pi/180;
%         
%         X = L2x - r.*cos(alpha).*sin(beta+beta_off);
%         Y = L2y - r.*sin(alpha).*sin(tilt+ tilt_off) + r.*cos(alpha).*cos(beta+beta_off).*cos(tilt+ tilt_off);
%         Z = L1z + L2z + r.*sin(alpha).*cos(tilt+ tilt_off) + r.*cos(alpha).*cos(beta+beta_off).*sin(tilt+tilt_off);
%     
        
    Weight = sqrt(Y/max(Y));
    %Weight = ones(size(Y));
    options = optimoptions(@lsqnonlin,'Display','iter');%'Algorithm','levenberg-marquardt'
    f= @(W)model_01b(W, r, beta, tilt, S);
    [W2,resnorm,residual,exitflag,output] = lsqnonlin(f,W,lb,ub,options);
    if(length(W2)>3)
      L1z = W2(1);
      L2x = W2(2);
      L2y = W2(3);
      L2z = W2(4);
      alpha1 = W2(5);
      alpha2 = W2(6);
      alpha3 = W2(7);
      alpha4 = W2(8);
      beta_off1 = W2(9);
      beta_off2 = W2(10);
      tilt_off = W2(11);
      pitch = W2(12);
      roll = W2(13);
      Loz = W2(14); 
    else
        pitch=W2(1);
        roll=W2(2);
        Loz=W2(3);
    end
    
    disp('Residual: '); disp(sum(residual.^2))
 %% graphics
    % Nominal
    plot3(X,Y,Z,'.')
    grid on
    hold on

    % calibrated
    alpha = [alpha1*ones(S(1),1);alpha2*ones(S(2),1);alpha3*ones(S(3),1);alpha4*ones(S(4),1)];
    beta_off =  [beta_off1+zeros(S(1),1);beta_off1+zeros(S(2),1); beta_off2+zeros(S(3),1);beta_off2+zeros(S(4),1)];
    Xcal = L2x*cos(roll) + L1z*sin(roll) + L2z*sin(roll) + r.*cos(tilt + tilt_off).*sin(alpha).*sin(roll) - r.*cos(alpha).*sin(beta+beta_off).*cos(roll) + r.*sin(tilt + tilt_off).*cos(alpha).*cos(beta+beta_off).*sin(roll);
    Ycal = L2y*cos(pitch) - L1z*cos(roll)*sin(pitch) - L2z*cos(roll)*sin(pitch) + L2x*sin(pitch)*sin(roll) - r.*sin(alpha).*(sin(tilt + tilt_off).*cos(pitch) + cos(tilt + tilt_off).*cos(roll).*sin(pitch)) + r.*cos(alpha).*cos(beta+beta_off).*(cos(tilt + tilt_off).*cos(pitch) - sin(tilt + tilt_off).*cos(roll).*sin(pitch)) - r.*cos(alpha).*sin(beta+beta_off).*sin(pitch).*sin(roll);
    Zcal = Loz+ L2y*sin(pitch) + L1z*cos(pitch)*cos(roll) + L2z*cos(pitch)*cos(roll) - L2x*cos(pitch)*sin(roll) - r.*sin(alpha).*(sin(tilt + tilt_off).*sin(pitch) - cos(tilt + tilt_off).*cos(pitch).*cos(roll)) + r.*cos(alpha).*cos(beta+beta_off).*(cos(tilt + tilt_off).*sin(pitch) + sin(tilt + tilt_off).*cos(pitch)*cos(roll)) + r.*cos(alpha).*cos(pitch).*sin(beta+beta_off).*sin(roll);
    plot3(Xcal,Ycal,Zcal,'.')
    view(-90,0)
    
    
    
