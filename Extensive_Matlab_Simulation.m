clc; clear; close all;

%% ================= COMMON VEHICLE PARAMETERS =================

Iz = 480;       
L  = 2.0;       
lf = 1.3;
lr = 0.7;
Tw = 1.2;       
rw = 0.22;      

Cf = 30000;     
Cr = 35000;

mu = 0.8;
g  = 9.81;

Tmax = 40;

dt = 0.01;
Tsim = 10;
N = Tsim/dt;

%% ================= FUNCTION HANDLE FOR SIMULATION =================

simulate_case = @(m,case_type) run_sim(m,case_type,Iz,L,lf,lr,Tw,rw,Cf,Cr,mu,g,Tmax,dt,N);

%% ================= CASE 1: NORMAL TURN =================

disp("Running Case 1: Normal Turning")
data1 = simulate_case(760,"normal");

%% ================= CASE 2: MU-SPLIT =================

disp("Running Case 2: μ-Split Acceleration")
data2 = simulate_case(760,"musplit");

%% ================= CASE 3: LOAD VARIATION =================

disp("Running Case 3: Load Variation")

data3_light = simulate_case(450,"normal");
data3_heavy = simulate_case(760,"normal");

%% ================= PLOTS =================

plot_case(data1,"Normal Turning Case");
plot_case(data2,"μ-Split Acceleration Case");

figure;
plot(data3_light.time,data3_light.gamma,'b','LineWidth',2); hold on;
plot(data3_heavy.time,data3_heavy.gamma,'r','LineWidth',2);
plot(data3_heavy.time,data3_heavy.gamma_ref,'k--','LineWidth',2);
grid on
xlabel("Time (seconds)")
ylabel("Yaw Rate (rad/s)")
title("Load Variation: Yaw Rate Comparison")
legend("Light Load","Heavy Load","Reference Yaw Rate")
text(2,max(data3_heavy.gamma_ref)*0.7,...
"Heavy vehicle responds slower but remains stable")

%% ================= SIMPLE 2D ANIMATION =================

animate_2D(data1.X,data1.Y,"Normal Turn 2D Animation");

%% ================= FUNCTION DEFINITIONS =================

function data = run_sim(m,case_type,Iz,L,lf,lr,Tw,rw,Cf,Cr,mu,g,Tmax,dt,N)

vx = 12; vy = 0; gamma = 0; psi = 0; X=0; Y=0;

delta = zeros(N,1);
Treq  = 25*ones(N,1);

if case_type=="normal"
    for k=200:600
        delta(k)=deg2rad(6);
    end
elseif case_type=="musplit"
    Treq(:)=30;
end

time = (0:N-1)*dt;

Xh=zeros(N,1); Yh=zeros(N,1);
gamma_h=zeros(N,1); gamma_ref_h=zeros(N,1);
TL_h=zeros(N,1); TR_h=zeros(N,1);

for k=1:N
    
    if vx<0.5; vx=0.5; end
    
    alpha_f = delta(k)-(vy+lf*gamma)/vx;
    alpha_r = -(vy-lr*gamma)/vx;
    
    Fyf = Cf*alpha_f;
    Fyr = Cr*alpha_r;
    
    Kus = (m/(L^2))*(lr/Cf - lf/Cr);
    gamma_ref = (vx/(L+Kus*vx^2))*delta(k);
    
    gamma_limit = mu*g/vx;
    gamma_ref = sign(gamma_ref)*min(abs(gamma_ref),abs(gamma_limit));
    
    e = gamma - gamma_ref;
    DeltaMz = -600*e;
    
    TR = Treq(k)/2 + (rw/Tw)*DeltaMz;
    TL = Treq(k)/2 - (rw/Tw)*DeltaMz;
    
    TR = max(min(TR,Tmax),-Tmax);
    TL = max(min(TL,Tmax),-Tmax);
    
    FxR = TR/rw; FxL = TL/rw;
    
    if case_type=="musplit"
        muL=0.2; muR=0.8;
        FxL=max(min(FxL,muL*m*g/2),-muL*m*g/2);
        FxR=max(min(FxR,muR*m*g/2),-muR*m*g/2);
    end
    
    vx_dot=(FxR+FxL)/m + vy*gamma;
    vy_dot=(Fyf+Fyr)/m - vx*gamma;
    gamma_dot=(lf*Fyf-lr*Fyr+(Tw/(2*rw))*(TR-TL))/Iz;
    
    vx=vx+vx_dot*dt;
    vy=vy+vy_dot*dt;
    gamma=gamma+gamma_dot*dt;
    psi=psi+gamma*dt;
    
    X=X+(vx*cos(psi)-vy*sin(psi))*dt;
    Y=Y+(vx*sin(psi)+vy*cos(psi))*dt;
    
    Xh(k)=X; Yh(k)=Y;
    gamma_h(k)=gamma;
    gamma_ref_h(k)=gamma_ref;
    TL_h(k)=TL; TR_h(k)=TR;
end

data.X=Xh; data.Y=Yh;
data.gamma=gamma_h;
data.gamma_ref=gamma_ref_h;
data.TL=TL_h; data.TR=TR_h;
data.time=time;
end

function plot_case(data,case_name)

figure;
plot(data.time,data.gamma,'b','LineWidth',2); hold on;
plot(data.time,data.gamma_ref,'r--','LineWidth',2);
grid on
xlabel("Time (seconds)")
ylabel("Yaw Rate (rad/s)")
title("Yaw Rate Tracking - "+case_name)
legend("Actual Yaw Rate","Reference Yaw Rate")
text(2,max(data.gamma_ref)*0.7,...
"If tracking is close → controller working")

figure;
plot(data.time,data.TL,'LineWidth',2); hold on;
plot(data.time,data.TR,'LineWidth',2);
grid on
xlabel("Time (seconds)")
ylabel("Motor Torque (Nm)")
title("Rear Motor Torque Distribution - "+case_name)
legend("Left Motor Torque","Right Motor Torque")
text(2,max(data.TR)*0.7,...
"Torque difference generates corrective yaw moment")

figure;
plot(data.X,data.Y,'LineWidth',2);
grid on
axis equal
xlabel("Longitudinal Position X (m)")
ylabel("Lateral Position Y (m)")
title("Vehicle Path - "+case_name)
text(mean(data.X),mean(data.Y),...
"Smooth curve = stable behavior")
end

function animate_2D(X,Y,title_name)

figure;
plot(X,Y,'k','LineWidth',1); hold on;
car = plot(X(1),Y(1),'ro','MarkerSize',10,'MarkerFaceColor','r');
grid on
axis equal
xlabel("Longitudinal Position X (m)")
ylabel("Lateral Position Y (m)")
title(title_name)

for k=1:10:length(X)
    set(car,'XData',X(k),'YData',Y(k));
    drawnow
end
end