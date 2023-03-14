% assignment 2 : recursvive inverse dynamics
% Lugano Giacomo mat. 5400573
% note: all the results are expressed in the local referance frame of each
% link, which follow the denavit hartenberg rule, is alligned with the
% principal inertial axis and the origin is located in the center of mass
% of each link.
%% exercise 2.1 without gravity:
clc
% settings:
l = [1, 0.8];
C = [0.5, 0, 0;
     0.4, 0, 0];
I_c(:,:,1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.4];
I_c(:,:,2) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.3];
m = [22; 19];
q = [0.349066, 0, 0, 0, 0.349066;
    0.698132, 0, 0, 0, 0.698132];
q_d = [0.2; 0.15];
q_dd = [0.1; 0.085];
g_flag = 0;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exercise 2.1 with gravity:
clc
% settings:
l = [1, 0.8];
C = [0.5, 0, 0;
     0.4, 0, 0];
I_c(:,:,1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.4];
I_c(:,:,2) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.3];
m = [22; 19];
q = [0.349066, 0, 0, 0, 0.349066;
    0.698132, 0, 0, 0, 0.698132];
q_d = [0.2; 0.15];
q_dd = [0.1; 0.085];
g_flag = 1;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exercise 2.2 without gravity:
clc
% settings:
l = [1, 0.8];
C = [0.5, 0, 0;
     0.4, 0, 0];
I_c(:,:,1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.4];
I_c(:,:,2) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.3];
m = [22; 19];
q = [pi/2, 0, 0, 0, pi/2;
    pi/4, 0, 0, 0, pi/4];
q_d = [-0.8; 0.35];
q_dd = [-0.4; 0.1];
g_flag = 0;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exercise 2.2 with gravity:
clc
% settings:
l = [1, 0.8];
C = [0.5, 0, 0;
     0.4, 0, 0];
I_c(:,:,1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.4];
I_c(:,:,2) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.3];
m = [22; 19];
q = [pi/2, 0, 0, 0, pi/2;
    pi/4, 0, 0, 0, pi/4];
q_d = [-0.8; 0.35];
q_dd = [-0.4; 0.1];
g_flag = 1;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exercise 3.1 without gravity:
clc
% settings:
l = [1, 0.8];
C = [0.5, 0, 0;
     0.4, 0, 0];
I_c(:,:,1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.4];
I_c(:,:,2) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.3];
m = [10; 6];
q = [0.349066, 0, 0, 0, 0.349066;
    -0.2, 1, 0, pi/2, 0];
q_d = [0.08; 0.03];
q_dd = [0.1; 0.01];
g_flag = 0;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exercise 3.1 with gravity:
clc
% settings:
l = [1, 0.8];
C = [0.5, 0, 0;
     0.4, 0, 0];
I_c(:,:,1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.4];
I_c(:,:,2) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.3];
m = [10; 6];
q = [0.349066, 0, 0, 0, 0.349066;
    -0.2, 1, 0, pi/2, 0];
q_d = [0.08; 0.03];
q_dd = [0.1; 0.01];
g_flag = 1;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exercise 3.2 without gravity:
clc
% settings:
l = [1, 0.8];
C = [0.5, 0, 0;
     0.4, 0, 0];
I_c(:,:,1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.4];
I_c(:,:,2) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.3];
m = [10; 6];
q = [2.0944, 0, 0, 0, 2.0944;
    -0.6, 1, 0, pi/2, 0];
q_d = [-0.4; -0.08];
q_dd = [-0.1; -0.01];
g_flag = 0;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exrecise 3.2 with gravity:
clc
% settings:
l = [1, 0.8];
C = [0.5, 0, 0;
     0.4, 0, 0];
I_c(:,:,1) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.4];
I_c(:,:,2) = [0, 0, 0;
    0, 0, 0;
    0, 0, 0.3];
m = [10; 6];
q = [2.0944, 0, 0, 0, 2.0944;
    -0.6, 1, 0, pi/2, 0];
q_d = [-0.4; -0.08];
q_dd = [-0.1; -0.01];
g_flag = 1;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exercise 4.1 without gravity:
clc
% settings:
l = [1, 0.8, 0.35];
C = [0.5, 0, 0;
     0.4, 0, 0;
     0.175, 0, 0];
I_c(:,:,1) = [0.2, 0, 0;
    0, 0.2, 0;
    0, 0, 0.8];
I_c(:,:,2) = [0.2, 0, 0;
    0, 0.2, 0;
    0, 0, 0.8];
I_c(:,:,3) = [0.08, 0, 0;
    0, 0.08, 0;
    0, 0, 0.1];
m = [20; 20; 6];
q = [0.349066, 0, -pi/2, 0.349066, 0;
    0.698132, 0, pi/2, 0, 0.698132;
    0.1745, 0, 0, 0, 0.1745];
q_d = [0.2; 0.15; -0.2];
q_dd = [0.1; 0.085; 0];
g_flag = 0;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end
%% exercise 4.1 with gravity:
clc
% settings:
l = [1, 0.8, 0.35];
C = [0.5, 0, 0;
     0.4, 0, 0;
     0.175, 0, 0];
I_c(:,:,1) = [0.2, 0, 0;
    0, 0.2, 0;
    0, 0, 0.8];
I_c(:,:,2) = [0.2, 0, 0;
    0, 0.2, 0;
    0, 0, 0.8];
I_c(:,:,3) = [0.08, 0, 0;
    0, 0.08, 0;
    0, 0, 0.1];
m = [20; 20; 6];
q = [0.349066, 0, -pi/2, 0.349066, 0;
    0.698132, 0, pi/2, 0, 0.698132;
    0.1745, 0, 0, 0, 0.1745];
q_d = [0.2; 0.15; -0.2];
q_dd = [0.1; 0.085; 0];
g_flag = 1;
F_ext = [0 ,0 ,0 , 0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0;
     0 ,0 ,0 ,0 ,0 ,0 ,0];
M_ext = [0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
% evaluation:
[t_eq, vr] = dynamic_processing(l, C, I_c, m, q, q_d, q_dd, g_flag, F_ext, M_ext);
% print the results:
fprintf('the generalized actuation torque and force vector is composed by: \n')
for i = 1 : size(l,2)
    if q(i,2) == 1
        fprintf('tau %i = %4.2f N \n', i, t_eq(i))
    else
        fprintf('tau %i = %4.2f Nm \n', i, t_eq(i))
    end
end
fprintf('while the vincular reaction forces and moment are: \n')
for i = 1 : size(l,2)
        fprintf('vr %i = [%4.2f N, %4.2f N,  %4.2f N,  %4.2f Nm,  %4.2f Nm,  %4.2f Nm] \n', i, vr(i,1), vr(i,2), vr(i,3), vr(i,4), vr(i,5), vr(i,6))
end


