M = readmatrix('../test/end_poses.csv')

% take first element of M
plot3(M(:,1),M(:,2),M(:,3),'o', 'MarkerFaceColor','#0072BD')

xlim([-0.3,0.3])
ylim([-0.3,0.3])
zlim([0,0.52])

set(get(gca, 'XLabel'), 'String', 'x[m]');
set(get(gca, 'YLabel'), 'String', 'y[m]');
set(get(gca, 'ZLabel'), 'String', 'z[m]');

%insert text boks at [0,0,0]
text(0,0,0.02,'Object position', 'FontWeight','bold')
text(0,0,0.33,'Light positions', 'FontWeight','bold')

    
set(get(gca, 'Title'), 'String', 'Light positions relative to object');
    