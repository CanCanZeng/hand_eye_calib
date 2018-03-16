%%

T_g0_gi = eye(4, 4); % 假设初始位置机械手末端坐标系与基座坐标系重合
T_c0_ci = eye(4, 4); % 同样也假设初始位置相机坐标系与标定板坐标系重合

T_g_c = [1 0 0 22;
              0 1 0 1;
              0 0 1 150;
              0 0 0 1];  % 假设 相机在机械手末端 正 x 方向上, 距离单位 1

num_of_frame = 20;
 bHg = zeros(4, 4, num_of_frame); % 机械手的一系列pose,(orientation, location)
 cHw = bHg; % 相机外参
          
for i = 1 : num_of_frame
    r = rand(3,1);
    R = rotationVectorToMatrix(r);
    t = rand(3,1);
    T_gj_gi = [R, t;
                    0 0 0 1]; % 假设从 i 时刻 机械手到 j 时刻机械手的位置是自变量
    T_g0_gj = T_g0_gi * inv(T_gj_gi); % 这是 j 时刻机械手的位置
    
    T_ci_gi = inv(T_g_c);
    T_gj_cj = T_g_c;
    T_gi_gj = inv(T_gj_gi);
    T_c0_cj = T_c0_ci * T_ci_gi * T_gi_gj * T_gj_cj; % j 时刻相机的位置
    
    T_cj_c0 = inv(T_c0_cj);
    T_cj_ci = T_cj_c0 * T_c0_ci; % i 时刻到 j 时刻相机的位置变化
    
    bHg(:,:,i) = T_g0_gj;
    cHw(:,:,i) = T_cj_c0;
end
    

gHc = handEye(bHg, cHw);
disp(gHc);

