% Inverse Kinematics Test
%rosshutdown;
%ur5 = ur5_interface();
%joint_offset = [-pi/2 -pi/2 0 -pi/2 0 0]';
%joints = [0 0 0 0 0 0]';
%g_S_T = ur5FwdKin(joints- joint_offset);
%g_baseK_S = [ROTZ(-pi/2) [0 0 0.0892]'; 0 0 0 1];  %transformation from keating base to {S}
%-90 degree rotation around z and up x 0.0892 
%baseKFrame = tf_frame('S','base_K',eye(4));
%baseKFrame.move_frame('S',inv(g_baseK_S));

%g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1]; %transformation from {T} to keating tool 
%-90 around x and 90 around y
%toolKFrame = tf_frame('T','tool_K',eye(4));
%toolKFrame.move_frame('T',g_T_toolK);

%g_des = g_baseK_S*g_S_T*g_T_toolK; %transformation from keating base to keating tool 
g_des = [0.9576, 0.04315, 0.2849, 0.524;     
  -0.04315, -0.9561, 0.2899, 0.3774;
   0.2849, -0.2899, -0.9137, 0.1845;
   0,0,0,1]
thetas = ur5InvKin(g_des)