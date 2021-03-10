clc;
clear all;
pcls = csvread('edge.csv');
% plot the data
plot3(pcls(:,1), pcls(:,2), pcls(:,3),'.');
hold on;
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-20, 20])
ylim([-20, 20])
zlim([-3 3])
title('edge')

figure
surf = csvread('surf.csv');
% plot the data
plot3(surf(:,1), surf(:,2), surf(:,3),'.r');
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-20, 20])
ylim([-20, 20])
zlim([-3 3])
title('surf')

figure
plot3(pcls(:,1), pcls(:,2), pcls(:,3),'.');
hold on;
plot3(surf(:,1), surf(:,2), surf(:,3),'.r');
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-20, 20])
ylim([-20, 20])
zlim([-3 3])
title('together')


%% preprocessing
% remove vehicle point clouds
id = sqrt(sum(pcls.^2,2)) > 3.0;
pcls = pcls(id,:);

% remove point clouds behine
id = pcls(:,1) > -1.0;
pcls = pcls(id,:);

% remove non-ground point clouds
idz = pcls(:,3) > -2.6 % & pcls(:,3) < -1.85;
pcls = pcls(idz,:);

% remove any data outside 20 meters radius
%id = sqrt(sum(pcls.^2,2)) <= 20.0;
%pcls = pcls(
% plot the data
plot3(pcls(:,1), pcls(:,2), pcls(:,3),'.');
hold on;

%% plot 3D ball
r = 3.0;
[x,y,z] = sphere(50);
x = x*r;
y = y*r;
z = z*r;
lightGrey = 0.9*[1 1 1]; % It looks better if the lines are lighter
surface(x,y,z,'FaceColor', 'none','EdgeColor',lightGrey)
hold on;

axis square
xlim([-20 20]);
ylim([-20 20]);
zlim([-20,20]);

%% ring container
ring = cell(1,30);
for i = 1:size(pcls,1)
    rid = getRingID(pcls(i,1), pcls(i,2), pcls(i,3));
    ring{rid} = [ring{rid}; pcls(i,:)];
end

i = 25;
angle = atan2(ring{i}(:,2), ring{i}(:,1)) .* 180/pi;
figure;
plot(angle, ring{i}(:,3), 'o-');
hold on;

f = fit(angle, ring{i}(:,3), 'smoothingspline', 'SmoothingParam', 0.005);
plot(f);

figure;
deriv = differentiate(f,angle);
plot(angle, deriv);
hold on
plot([-80,100], [0,0], '--');



%for i = 1:length(ring)
%    angle = atan2(ring{i}(:,2), ring{i}(:,1)) .* 180/pi;
%    figure;
%    plot(angle, ring{i}(:,3),'o-');
%end

n = size(ring{26}(:,3),1);
% get xy-plane angle for each points

s = spline(1:n, ring{26}(:,3));

