function [t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext)
% preliminary definition:
% link's lenghts: assign for each link his lenght which is the distance
% between the origin of the local reference frame of the link and the
% position of the origin of the next one in the chain (in m) for istance:
%                 l = [1, 0.8];
% number of link:
n = size(l,2);
% center of mass of each link expressed in the local reference frame localised in the point Pi of the link (in m) for istance:
%                 C = [0.5, 0, 0;
%                      0.4, 0, 0];
% define the inertial matrix for each link for istance:
%                 I_c(:,:,1) = [0, 0, 0;
%                               0, 0, 0;
%                               0, 0, 0.4];
%                 I_c(:,:,2) = [0, 0, 0;
%                               0, 0, 0;
%                               0, 0, 0.3];
% mass of each link (in kg) for istance:
%                 m = [22; 19] ;
% configuration vector and joint type: in which the first column contain the configuration distance (in m) in case of prismatic joint, the configuration
% angle (in rad) in case of revolute joint; the second column contain the
% joint type ( 0 for revolute joint, 1 for prismatic joint); the last three
% column contain the angles for the rotation matrix rispectively around x,
% y, z between the link l-1 and l( where l is the line number) for istance:
%                 q = [0.349066, 0, 0, 0, 0.349066;
%                      0.698132, 0, 0, 0, 0.698132];
% velocity configuration vector: contain the configuration speed of the
% joints in m/s in case of prismatic joint and rad/s in case of revolute
% joint for istance:
%                 q_d = [0.2; 0.15];
% accelleration configuration vector: contain the configuration
% acelleration of the joints in m/s^2 in case of prismatic joint and
% rad/s^2 in case of revolute joint, for istance:
%                 q_dd = [0.1; 0.085];
% gravity flag: 1 to set the gravity on, 0 to set the gravity off, for istance:
%                 g_flag = 0; 
% external forces acting on the robot: in the vector the first three
% coloumns contain the force components along the axis of the absolute
% reference frame (in N), the fourth coloumn contain the number of the link on
% which the force act, the last three coloumns contain the position where
% the force act in the local reference frame of the link centered in the
% center of mass, for istance:
%                 F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
%                          0 ,0 ,0 ,0 ,0 ,0 ,0;
%                          0 ,0 ,0 ,0 ,0 ,0 ,0];
% external torque acting on the robot:in the vector the first three
% coloumns contain the torque components along the axis of the absolute
% reference frame (in N*m), the fourth coloumn contain the number of the link on
% which the torque act, for istance:
%                 M_ext = [0, 0, 0, 0;
%                          0, 0, 0, 0;
%                          0, 0, 0, 0];
% definition of the relative rotational matrises:
Rr(:,:,n+1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0;];
for i = 1 : n
     Rr(:,:,i) = [cos(q(i,5))*cos(q(i,4)), -sin(q(i,5))*cos(q(i,3)) + cos(q(i,5))*sin(q(i,4))*sin(q(i,3)), sin(q(i,5))*sin(q(i,3)) + cos(q(i,5))*sin(q(i,4))*cos(q(i,3));
           sin(q(i,5))*cos(q(i,4)), sin(q(i,5))*sin(q(i,4))*sin(q(i,3)) + cos(q(i,5))*cos(q(i,3)), sin(q(i,5))*sin(q(i,4))*cos(q(i,3)) - cos(q(i,5))*sin(q(i,3));
           -sin(q(i,4)), cos(q(i,4))*sin(q(i,3)), cos(q(i,4))*cos(q(i,3))];
end     
% definition of the absolute rotational matrises:
for j = 1 : n
    if j == 1
        Ra(:,:,1) = Rr(:,:,1);
    else
        Ra(:,:,j) = Ra(:,:,j-1) * Rr(:,:,j);
    end
end
% forward recursive progrm:
% inizialization of the kinematic parameters for the root link:
% in case it's connected with the ground by a prismatic joint:
if q(1,2) == 1
    % evaluation of the positions:
    r_cp(1,:) = - (C(1,:) + [0, 0, q(1,1)]);
    r_pq(1,:) = [0, 0, q(1,1) + l(1)];
    r_cq(1,:) = r_pq(1,:) + r_cp(1,:);
    % evaluation of the angular and linear velocities:
    w_p0(1,:) = [0, 0, 0];
    v_p0(1,:) = [0, 0, q_d(1,1)];
    v_c0(1,:) = v_p0(1,:);
    % evaluation of the angular and linear accellerations:
    w_p0d(1,:) = [0, 0, 0];
    v_p0d(1,:) = [0, 0, q_dd(1,1)];
    v_c0d(1,:) = v_p0d(1,:); 
    % in case it's connected with the ground by a revolute joint:
else
    % evaluation of the positions:
    r_cp(1,:) = - C(1,:);
    r_pq(1,:) = [l(1), 0, 0];
    r_cq(1,:) = r_pq(1,:) + r_cp(1,:);
    % evaluation of the angular and linear velocities:
    w_p0(1,:) = [0, 0, q_d(1,1)];
    v_p0(1,:) = [0, 0, 0];
    v_c0(1,:) = (cross(r_cp(1,:)', w_p0(1,:)'))';
    % evaluation of the angular and linear accellerations:
    w_p0d(1,:) = [0, 0, q_dd(1,1)];
    v_p0d(1,:) = [0, 0, 0];
    v_c0d(1,:) = cross(r_cp(1,:)', w_p0d(1,:)')' + (cross(w_p0(1,:)', (cross(r_cp(1,:)', w_p0(1,:)'))))';
end
% evaluation for the following links:
for k = 2 : n
    % in case it's connected with the first link by a prismatic joint:
    if q(k,2) == 1
        % evaluation of the positions:
        r_cp(k,:) = - (C(k,:) + [0, 0, q(k,1)]);
        r_pq(k,:) = [0, 0, q(k,1) + l(k)];
        r_cq(k,:) = r_pq(k,:) + r_cp(k,:);
        % evaluation of the angular and linear velocities:
        w_p0(k,:) = (Rr(:,:,k)' * w_p0(k-1,:)')';
        v_p0(k,:) = (Rr(:,:,k)' * v_p0(k-1,:)')' + (cross(Rr(:,:,k)' * w_p0(k-1,:)', r_pq(k-1,:)'))' + [0, 0, q_d(k,1)];
        v_c0(k,:) = v_p0(k,:) + (cross(r_cp(k,:)', w_p0(k,:)'))';
        % evaluation of the angular and linear accellerations:
        w_p0d(k,:) = (Rr(:,:,k)' * w_p0d(k-1,:)')';
        v_p0d(k,:) = (Rr(:,:,k)' * v_p0d(k-1,:)')' + (Rr(:,:,k)' * cross(w_p0d(k-1,:)', r_pq(k-1,:)'))' + (cross(Rr(:,:,k)' * w_p0(k-1,:)', (Rr(:,:,k)' * cross(w_p0(k-1,:)', r_pq(k-1,:)'))))' + 2 * q_d(k,1) * (cross(Rr(:,:,k)' * w_p0(k-1,:)', [0; 0; 1]))' + [0, 0, q_dd(k,1)];
        v_c0d(k,:) = v_p0d(k,:) + cross(r_cp(k,:)', w_p0d(k,:)')' + (cross(w_p0(k,:)',(cross(r_cp(k,:)', w_p0(k,:)'))))';
        % in case it's connected with the first link by a revolute joint:
    else
        % evaluation of the positions:
        r_cp(k,:) = - C(k,:);
        r_pq(k,:) = [l(k), 0, 0];
        r_cq(k,:) = r_pq(k,:) + r_cp(k,:);
        % evaluation of the angular and linear velocities:
        w_p0(k,:) = (Rr(:,:,k)' * w_p0(k-1,:)')' + [0, 0, q_d(k,1)];
        v_p0(k,:) = (Rr(:,:,k)' * v_p0(k-1,:)')' + (cross(Rr(:,:,k)' * w_p0(k-1,:)', r_pq(k-1,:)'))';
        v_c0(k,:) = v_p0(k,:) + (cross(r_cp(k,:)', w_p0(k,:)'))';
        % evaluation of the angular and linear accellerations:
        w_p0d(k,:) = (Rr(:,:,k)' * w_p0d(k-1,:)')' + (cross(Rr(:,:,k)' * w_p0(k-1,:)', [0; 0; 1]))' * q_d(k,1) + [0, 0, q_dd(k,1)];
        v_p0d(k,:) = (Rr(:,:,k)' * (v_p0d(k-1,:)' + cross(w_p0d(k-1,:)', r_pq(k-1,:)') + cross(w_p0(k-1,:)',((cross(w_p0(k-1,:)', r_pq(k-1,:)'))))));
        v_c0d(k,:) = v_p0d(k,:) + cross(r_cp(k,:)', w_p0d(k,:)')' + (cross(w_p0(k,:)',(cross(r_cp(k,:)', w_p0(k,:)'))))';
    end
end
% inverce recursive program:
% gravity on or off:
if g_flag == 1
    g = [0, -9.81, 0];
else
    g = [0, 0, 0];
end
% starting evaluation:
t_eq = [];
vr = [];
for k = n : -1 : 1
    F_1(:,n+1) = [0, 0, 0];
    M(:,n+1) = [0, 0, 0];
    Fc(k,:) = [0, 0, 0];
    Mc(k,:) = [0, 0, 0];
    F(:,n+1) = [0, 0, 0];
    for w = 1 : size(F_ext,1) %for each link
        if F_ext(w,4) == k
            F3 = [F_ext(w,1); F_ext(w,2); F_ext(w,3)];
            Fc(k,:) = Fc(k,:) + ((Ra(:,:,k)') * F3)'; % summ all the forces acting on it
            Rcp = [F_ext(w,5), F_ext(w,6), F_ext(w,7)];
            Mc(k,:) = Mc(k,:) + (cross(Rcp',((Ra(:,:,k)') * F3)))'; % evaluate the external moment produced by the external force and his position
        end
    end
    for w = 1 : size(M_ext,1)
        if M_ext(w,4) == k
            T3 = [M_ext(w,1); M_ext(w,2); M_ext(w,3)];
            Mc(k,:) = Mc(k,:) +((Ra(:,:,k)') * T3)'; % adding the external moment
        end
    end
    % evaluate the force and moments exchanged in the joint
    F(:,k) = m(k) * v_c0d(k,:)' - m(k) * (Ra(:,:,k)' * g') - Fc(k,:)' + Rr(:,:,k+1) * F(:,k+1);
    M(:,k) = I_c(:,:,k) * w_p0d(k,:)' + cross(w_p0(k,:)', I_c(:,:,k) * w_p0(k,:)') + Rr(:,:,k+1) * M(:,k+1) + cross(r_cq(k,:)', Rr(:,:,k+1) * F(:,k+1)) - Mc(k,:)' - cross(r_cp(k,:)', F(:,k));
    % summarise and group the actuator action
    if q(k,2) == 0
        t_eq = [M(3, k); t_eq];
        vr = [F(1, k), F(2, k), F(3, k), M(1, k), M(2, k), 0; vr];
    elseif q(k,2) == 1
        t_eq = [F(3, k); t_eq];
        vr = [F(1, k), F(2, k), 0, M(1, k), M(2, k), M(3, k); vr];
    end
end
end