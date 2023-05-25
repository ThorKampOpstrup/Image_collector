%this file plots the generated frames

%load in data from csv file
pos_obj = [0.1818255165440956, -0.5101149287882081, 1.1227379674121187]
cam_pos = [0.1818255165440956, -0.5101149287882081, 1.1227379674121187]
%pos_obj = [0 0 0]


qo = quaternion([0 0 0],'eulerd','XYZ','frame');
po=pos_obj

qc = quaternion([0 0 0],'eulerd','XYZ','frame');
pc = cam_pos


end_poses = readmatrix('../test/end_poses.csv');


hold on
patch = poseplot(qo, po, ScaleFactor=0.05, DisplayName='obejct');
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)");
zlim([1.08,1.45])
%patch = poseplot(qc, pc, ScaleFactor=0.04, DisplayName='cam')
%loop though each frame
for i = 1:size(end_poses,1)
    %hold on
    pos = end_poses(i,1:3)
    rotation_vec = end_poses(i,4:6);
    axan = [rotation_vec/norm(rotation_vec), norm(rotation_vec)]
    %axan = [end_poses(i,4:6), 1]
    qe = axang2quat(axan);
    qe = quaternion(qe);
    
    %plot the frame
    patch = poseplot(qe, pos, ScaleFactor=0.065);

    %pause for 0.1 seconds
    %pause(0.1);
end

text(0.2, -0.5, 1.2, 'Object pose', 'FontSize', 10)
text(0.1, -0.5, 1.45, 'Light poses', 'FontSize', 10)
title('Light poses')
hold off
%hold off