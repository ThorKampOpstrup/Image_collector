pos_obj = [0.3, 0.3, 0.1]
pos_tcp = [1.1, 1.1, 1.1]

qb = quaternion([0 0 0],'eulerd','XYZ','frame');
pb=pos_obj;


%hold on

% ylim([-3 3])
% xlim([-3 3])
xlabel("North-x (m)")
ylabel("East-y (m)")
zlabel("Down-z (m)");

hold on

for angle = 0:45:360
    % ql = calulate_quaternion(pos_obj,pos_tcp,angle);
    % set(patch,Orientation=qe,Position=pos_tcp); 
    % drawnow
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rotation_around_z = deg2rad(angle);

    dir_vec=pos_obj-pos_tcp;
    
    tmp_dist=sqrt(dir_vec(1)^2+dir_vec(2)^2);
    rot_v = atan(dir_vec(3)/(tmp_dist))*-1;%the angle of the light to the object
    rot_h= atan(dir_vec(1)/dir_vec(2)); % angle of the light to the object around a circle
    
    rot_v = deg2rad(90+rad2deg(rot_v));
    
    rx = rot_v;
    ry = 0;
    rz = -rot_h;
    
    rot_to_allign = [rz ry rx];
    rotm_to_allign = eul2rotm(rot_to_allign, 'ZYX');
    %ql = rotm2quat(rotm_to_allign)
    %ql = quaternion(ql)
    
    rot_about_z = [rotation_around_z, 0, 0,];
    rotm_about_z = eul2rotm(rot_about_z, 'ZYX');
    
    rotm_combined = rotm_to_allign*rotm_about_z;
    
    test = rotm2axang(rotm_combined);
    test = test(:,1:3)*test(4);
    
    angle_axis=rotm2axang(rotm_combined);
    angle_axis=angle_axis(:,1:3)*angle_axis(4);

    ql_rotated = rotm2quat(rotm_combined);
    ql_rotated = quaternion(ql_rotated);
    ql=ql_rotated

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %ql = calulate_quaternion(pb, pos_tcp, angle);
    patch = poseplot(qb,pb, ScaleFactor=0.5);
    patch = poseplot(ql,pos_tcp, ScaleFactor=0.5);
    pause(0.1);

end
% poseplot(qb,pb);
% hold on
% poseplot(ql_rotated,pl);
% %set(gca, 'ZDir','normal')
% ylim([-2 5])
% xlim([-2 5])
% set(gca, 'ZDir','normal')
% xlabel("North-x (m)")
% ylabel("East-y (m)")
% zlabel("Down-z (m)");
% hold off