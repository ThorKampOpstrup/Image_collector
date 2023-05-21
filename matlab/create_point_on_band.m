%from https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
clc;
clear;

number_of_points = 100
dist = 0.30

points = []
phi = pi * (sqrt(5) - 1)
%Modify phi


min_angle = pi/8
max_angle = pi/3


minz = sin(min_angle)*dist
maxz = sin(max_angle)*dist

%z = 0.1:((0.1000001-0.1)/(number_of_points-1)):0.100001;
step_size_z = (maxz-minz)/number_of_points

for i=0:number_of_points-1
    %z = 0.5-((i / (number_of_points - 1)) * 1);  % y goes from 1 to -1
    z = minz+(i*step_size_z)
    angle_to_object = asin(z/dist);  % radius at y
    radius = sqrt(dist^2-z^2);
    theta = phi*i;  % golden angle increment

    x = cos(theta) * radius;
    y = sin(theta) * radius;

    points = [points; x y z];
end

plot3(points(:,1),points(:,2),points(:,3),'o')
zlim([0,dist])
ylim([-dist,dist])
xlim([-dist,dist])