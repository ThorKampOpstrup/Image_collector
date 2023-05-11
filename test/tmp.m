poses = readmatrix('auto_poses.csv')

legend on
rot_vecors = poses(:,4:6);

% calulate angle axis for all rot_vectors
axan = zeros(length(rot_vecors), 4);
quat = axan;
eul = zeros(length(rot_vecors), 3);
for i = 1:1:length(rot_vecors)
    axan(i,:) = [rot_vecors(i,:)/norm(rot_vecors(i,:)), norm(rot_vecors(i,:))];
    quat(i,:) = axang2quat(axan(i,:));
    q = quaternion(quat(i,:))
    eul(i,:) = quat2eul(q);

end


plot(eul(:,3))