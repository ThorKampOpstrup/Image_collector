qb = quaternion([0,0,0],'eulerd','XYZ','frame');
pb=[0 0 0]


object_pose = pb
rotation_around_z = deg2rad(10);

%tmp = 180-angle_to_object
pl=[1 2 3]
%pos=eye(3).*pl
dir_vec=object_pose-pl
%dir_vec = dir_vec/norm(dir_vec)

tmp_dist=sqrt(dir_vec(1)^2+dir_vec(2)^2)
rot_v = atan(dir_vec(3)/(tmp_dist))*-1%the angle of the light to the object
rot_h= atan(dir_vec(1)/dir_vec(2)) % angle of the light to the object around a circle

rot_v = deg2rad(90+rad2deg(rot_v))

%rx = rot_v * cos(rot_h)
%ry = rot_v * sin(rot_h)

rx = rot_v
ry = 0
rz = -rot_h

%ql = quaternion([rad2deg(rz), rad2deg(rx), rad2deg(ry)],'eulerd','ZXY','frame')

%tmp=rad2deg(quat2eul(ql,'ZYZ'))
%rx_updated= tmp(3)
%ry_updated
%quat2eul(ql)
%ql_rotated = quaternion(tmp,'eulerd','YXZ','frame')
%ql_tmp = quaternion([rotation_around_z, 0, 0],'eulerd','ZXY','frame')
%ql_rotated = ql_tmp + ql
rot_to_allign = [rz ry rx]
rotm_to_allign = eul2rotm(rot_to_allign, 'ZYX')
ql = rotm2quat(rotm_to_allign)
ql = quaternion(ql)

rot_about_z = [rotation_around_z, 0, 0,]
rotm_about_z = eul2rotm(rot_about_z, 'ZYX')

rotm_combined = rotm_to_allign*rotm_about_z

test = rotm2axang(rotm_combined);
test = test(:,1:3)*test(4)

angle_axis=rotm2axang(rotm_combined)
angle_axis=angle_axis(:,1:3)*angle_axis(4)

ql_rotated = rotm2quat(rotm_combined)
ql_rotated = quaternion(ql_rotated)


%set(gca, 'ZDir','normal')
poseplot(qb,pb);
hold on
poseplot(ql_rotated,pl);
%set(gca, 'ZDir','normal')
ylim([-2 5])
xlim([-2 5])
set(gca, 'ZDir','normal')
xlabel("North-x (m)")
ylabel("East-y (m)")
zlabel("Down-z (m)");
hold off