%M = readmatrix('../test/end_poses.csv')

M = readmatrix('/home/thor/Documents/8.Semester/Project/Image_collector/test/end_poses.csv')
% take first element of M
% delete elements on M where 1 element is <0
%M = M(M(:,1) >= 0,:)  
plot3(M(:,1),M(:,2),M(:,3),'o', 'MarkerFaceColor','#0072BD')
hold on 
%plot last element of M with orange color
plot3(M(end,1),M(end,2),M(end,3),'o', 'MarkerFaceColor','#D95319')

%xlim([-0.3,0.3])
%ylim([-0.3,0.3])
%zlim([0,0.6])

set(get(gca, 'XLabel'), 'String', 'x[m]');
set(get(gca, 'YLabel'), 'String', 'y[m]');
set(get(gca, 'ZLabel'), 'String', 'z[m]');

%insert text boks at [0,0,0]
text(0,0,0.02,'Object position', 'FontWeight','bold')
text(0,0,0.23,'Light positions', 'FontWeight','bold')

    
set(get(gca, 'Title'), 'String', 'Light positions relative to object');
