function draw_bicycle(x,y,theta,zeta)
% the length of the vehicle
front_length = 1; 
rear_length =1.5;
L = front_length+rear_length;

% Tire radius
r =0.3;

% Midpoint of front wheel
front_center = [x+front_length*cosd(theta),y+front_length*sind(theta)];
front_wheel1 = [front_center(1)-r*cosd(zeta),front_center(2)-r*sind(zeta)];
front_wheel2 = [front_center(1)+r*cosd(zeta),front_center(2)+r*sind(zeta)];

rear_center = [x-rear_length*cosd(theta),y-rear_length*sind(theta)];
rear_wheel1 = [rear_center(1)-r*cosd(theta),rear_center(2)-r*sind(theta)];
rear_wheel2 = [rear_center(1)+r*cosd(theta),rear_center(2)+r*sind(theta)];

wheelbase = [front_center;rear_center];
front_wheel = [front_wheel1;front_wheel2];
rear_wheel = [rear_wheel1;rear_wheel2];

figure (2)
plot(x,y,'ro','MarkerFaceColor','r');
hold on
plot(wheelbase(:,1),wheelbase(:,2),'-g','LineWidth',1.5);
plot(front_wheel(:,1),front_wheel(:,2),'-k','LineWidth',4.5);
plot(rear_wheel(:,1),rear_wheel(:,2),'-b','LineWidth',4.5);
%title({['\theta =',num2str(theta),', \beta =',num2str(beta)];['x=',num2str(x),',y=',num2str(y)]});
xlabel('X(m)','FontSize',14);
ylabel('Y(m)','FontSize',14);
grid on;

end