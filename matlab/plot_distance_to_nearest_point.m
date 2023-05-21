M = readmatrix('../test/end_poses.csv')
M=M(:,1:3)
M=M(1:length(M)-1,:)

distances = []

for i=1:length(M)
    tmp_dist = 10000;
    for j=1:length(M)
        tmptmpdist = norm(M(i,:) - M(j,:));
        if tmptmpdist<tmp_dist && i ~= j
            tmp_dist = tmptmpdist;
        end
    end
    tmp_dist
    if tmp_dist < 10000
        distances = [distances, tmp_dist];
    end
end

plot(distances)
xlabel('Point number')
ylabel('Distance to closest point[m]')
set(get(gca, 'Title'), 'String', 'Distance to closest point');
