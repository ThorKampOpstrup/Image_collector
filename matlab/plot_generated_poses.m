%this file plots the generated frames

%load in data from csv file
pos_obj = [0.3587039230223347, -0.5018220263120732, -0.06167880050691653]


qo = quaternion([0 0 0],'eulerd','XYZ','frame');
po=pos_obj


end_poses = readmatrix('../test/auto_poses.csv');

hold on
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
    patch = poseplot(qe, pos, ScaleFactor=0.05);
    patch = poseplot(qo, po, ScaleFactor=0.05);
    %pause for 0.1 seconds
    %pause(0.1);
end
hold off
%hold off