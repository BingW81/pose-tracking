clear
clc

path_point=textread('predict_point.txt');
[x_p,y_p,curv_clockwise]=clockwise_path_point(path_point);

% Initial state
x=-200;
y=0;
v=0;
theta=90; 
zeta=theta; 

%trajectory(x_p,y_p,x,y);

step=500;

x_goal=x_p(1);
y_goal=y_p(1);
theta_goal=90;

x_m=zeros(step,1);
x_m(1)=x;
y_m=zeros(step,1);
y_m(1)=y;
theta_m=zeros(step,1);
theta_m(1)=theta;
zeta_m=zeros(step,1);
zeta_m(1)=zeta;

deta_x=zeros(step,1);
deta_y=zeros(step,1);
rou=zeros(step,1);
alpha=zeros(step,1);
beta=zeros(step,1);

deta_x(1)=abs(x-x_goal);
deta_y(1)=abs(y-y_goal);
rou(1)=sqrt(deta_x(1)^2+deta_y(1)^2);
alpha(1)=atan(deta_y(1)/deta_x(1))-theta_m(1);
beta(1)=-theta_m(1)-alpha(1)-theta_goal;
v=zeros(step,1);

T=0.01;
time=(1:step)*T;

for i=1:step
    draw_bicycle(x_m(i),y_m(i),theta_m(i),zeta_m(i));
    
    deta_x(i)=x_m(i)-x_goal;
    deta_y(i)=y_m(i)-y_goal;
    rou(i)=sqrt(deta_x(i)^2+deta_y(i)^2);
    alpha(i)=atand(deta_y(i)/deta_x(i))-theta_m(i);
    beta(i)=-theta_m(i)-alpha(i)+theta_goal;
    
    [v(i),gamma(i)]=controller(i,rou,alpha,beta,T);
   
    [x_m(i+1),y_m(i+1),theta_m(i+1),zeta_m(i+1)]=update_motion(i,v,gamma,x_m,y_m,theta_m,zeta_m,T);
end

movingPoint_dis_velocity_gamma(x_m,y_m,time,rou,v,gamma,step);


function movingPoint_dis_velocity_gamma(x_m,y_m,time,rou,v,gamma,step)
figure (3)

plot(x_m(:),y_m(:),'-k');
title('the path');
title('planning path');
xlabel('X(m)','FontSize',14);
ylabel('Y(m)','FontSize',14);

figure (4)
subplot(3,1,1)
plot(time(:),rou(1:step),'-k');
title('distance error','FontSize',14);
xlabel('Time(s)','FontSize',10);
ylabel('distance(m)','FontSize',10);
subplot(3,1,2)
plot(time(:),v(1:step),'-k');
title('velocity','FontSize',14);
xlabel('Time(s)','FontSize',10);
ylabel('v(m/s)','FontSize',10);
subplot(3,1,3)
plot(time(:),gamma(1:step),'-k');
title('\gamma','FontSize',14);
xlabel('Time(s)','FontSize',10);
ylabel('degree','FontSize',10);

end


function [v,gamma]=controller(i,rou,alpha,beta,T)
kp_rou=0.5;
ki_rou=0.03;
kd_rou=0.03;
kp_alpha=1.7;
ki_alpha=0.03;
kd_alpha=0.03;
kp_beta=-1.5;
ki_beta= -0.05;
kd_beta= -0.05;

int_rou=rou(i)+sum(rou(1:i-1))*T;
int_alpha=alpha(i)+sum(alpha(1:i-1))*T;
int_beta=beta(i)+sum(beta(1:i-1))*T;

if i>=2
    diff_rou=(rou(i)-rou(i-1))/T;
    diff_alpha=(alpha(i)-alpha(i-1))/T;
    diff_beta=(beta(i)-beta(i-1))/T;
else 
    diff_rou=rou(i)/T;
    diff_alpha=alpha(i)/T;
    diff_beta=beta(i)/T;
end

v=kp_rou*rou(i)+ki_rou*int_rou;

gamma=kp_alpha*alpha(i)+ki_alpha*int_alpha+...
      kp_beta*beta(i)+ki_beta*int_beta;
if gamma<=-45
   gamma=-45;
elseif gamma>=45
   gamma=45;
end

end

function [x_m,y_m,theta_m,zeta_m]=update_motion(i,v,gamma,x_m,y_m,theta_m,zeta_m,T)
front_length = 1;

x_dot=v(i)*cosd(theta_m(i)+gamma(i));
y_dot=v(i)*sind(theta_m(i)+gamma(i));
theta_dot=v(i)*sind(gamma(i))/front_length;

x_m=x_m(i)+x_dot*T;
y_m=y_m(i)+y_dot*T;
theta_m=theta_m(i)+theta_dot*T;
zeta_m=zeta_m(i)+gamma(i)*T;

if zeta_m>=60
   zeta_m=60;
elseif zeta_m<=-60
   zeta_m=-60;
end

end


function trajectory(x_p,y_p,x,y)
figure (1)
plot(x_p,y_p,'.b',x,y,'ro','MarkerFaceColor','r');
title('planning path');
xlabel('X(m)','FontSize',14);
ylabel('Y(m)','FontSize',14);
grid on;

end