M = readmatrix('../test/end_poses.csv')

% take first element of M
plot3(M(:,1),M(:,2),M(:,3),'o')