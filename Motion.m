%% 轨迹规划（局部路径规划）

% feer 0.3, feer 衰减0.7, 80s, 9991分
function action = Motion(map, agent, anchor, feer, score_type)
    % 恐惧值
    if feer < 0.3
        cfg = struct(...
            'v_min', 0, 'v_max', 1, 'w_min', -0.5, 'w_max', 0.5,...
            'dt',0.1, 'dv', 0.1, 'dw', 0.01, 'sim_T', 2,...
            'a', 1, 'b', 3, 'c', 2, 'score_type', score_type ...
         );
    else
        cfg = struct(...
            'v_min', -1, 'v_max', 1, 'w_min', -0.5, 'w_max', 0.5,...
            'dt',0.1, 'dv', 0.1, 'dw', 0.01, 'sim_T', 2,...
            'a', 0.2, 'b', 3, 'c', 2, 'score_type', score_type ...
         );
    end
    [v, w] = dwa(map, cfg, agent, anchor);
    action = [v, w];
end
%% dwa算法
function [v, w] = dwa(map, cfg, agent, anchor)
% 动态窗口方法的主要流程
    % 生成动作空间的评估表
    V = 1;
    W = 2;
    DIST = 3;
    VELOCITY = 4;
    HEADING = 5;
    SCORE = 6;
    tbl = eval_func(agent, cfg, map, anchor);
    
    best = tbl(tbl.score == max(tbl.score), :);
    if size(best) > 1
        best = best(1,:);
    end
    % 获取最优动作
    v = best.v;
    w = best.w;

%     con = tbl(tbl.v == 1 & tbl.w == -w, :);
% 
%     disp(join([' x ', num2str(agent.x), ' y ', num2str(agent.y)]));
%     disp(join([" v ",num2str(v)," w ",num2str(w),...
%         " h ",num2str(agent.h), " heading ", num2str(best.heading), " dist ", num2str(best.dist),...
%         " score ", num2str(best.score)]));
%     
%     disp(join([" v ",num2str(v)," w ",num2str(-w),...
%         " h ",num2str(agent.h), " heading ", num2str(con.heading), " dist ", num2str(con.dist),...
%         " score ", num2str(con.score)]));
%     disp(" ");
    figure(1);
    persistent hdl;
   
    if isempty(hdl)
        hdl = scatter(best.traj.x, best.traj.y, 'Marker', '.');
    else
        set(hdl, 'XData', best.traj.x(:), 'YData', best.traj.y(:));
    end
   
end
%% 轨迹采样
function [info, traj] = gen_traj(v, w, st, T)
% st struct(x,y,h,v,w)
% T估计时间，不必太长
% info: struct(ex,ey,eh),轨迹的起点和终点姿态
% traj: struct(x[], y[], h[]),轨迹
    tspan = 0:0.1:T;
    tspan = tspan.';
    x0 = st.x;
    y0 = st.y;
    h0 = st.h;
    % 轨迹
    if w ~= 0
        fh = w.*tspan + h0;
        fx = x0 + v/w.*sin(fh) - v/w*sin(h0);
        fy = y0 - v/w.*cos(fh) + v/w*cos(h0);
    else
        fh = w.*tspan + h0;
        fx = x0 + v*cos(h0).*tspan;
        fy = y0 + v*sin(h0).*tspan;
    end
    % 终点
    eh = fh(end);
    ex = fx(end);
    ey = fy(end);

    traj = struct();
    traj.x = fx;
    traj.y = fy;
    traj.h = fh;

    info = struct();
    info.ex = ex;
    info.ey = ey;
    info.eh = eh;
end

%% 衡量函数
function score = heading(x, y, h, anchor)
% 和目标的相对角度
    anchor_vec = [anchor.x ; anchor.y];
    cur_vec = [x; y];
    relative = anchor_vec - cur_vec;
    if norm(relative) > 1e-2
        pos_vec = [cos(h); sin(h)];
        relative = relative/norm(relative);
        agl = acos(dot(relative, pos_vec));
    else
        agl = 0;
    end
    score = 2*pi - abs(agl);
end

% function score = distance(x, y, traj, global_map)
function score = distance(x, y, traj, global_map)
% 避障，和障碍物的距离
    sz = [50, 50];
    id = find(global_map ~= 0);
    [ob_x, ob_y] = ind2sub(sz, id);
    di = sqrt((ob_x - x).^2 + (ob_y - y).^2);
    idx_di = di < 5;
    if ~isempty(di(idx_di))
        % 小范围内存在障碍物
        %? score = min(di(idx_di));
        ob = [ob_x(idx_di), ob_y(idx_di)];
        tj = [traj.x , traj.y];
        D = pdist2(ob, tj);
        score = min(min(D));
    else
        % 本范围内没有障碍物
        score = 10.2;
    end
end

% function score = velocity(v)
% 速度不需要使用函数  
% end

function tbl = eval_func(st, cfg, map, anchor)
    % 生成轨迹并获取分数
    % 可以考虑向量化加速
    V = 1;
    W = 2;
    DIST = 3;
    VELOCITY = 4;
    HEADING = 5;
    SCORE = 6;

    score_tbl = [];
    node_tbl = [];
    v_range = cfg.v_min:cfg.dv:cfg.v_max;
    w_range = cfg.w_min:cfg.dw:cfg.w_max;
    % 轨迹生成
    for v=v_range
        for w=w_range
            [info, traj] = gen_traj(v, w, st, cfg.sim_T);

            dista = distance(st.x, st.y, traj, map);
            velocity = abs(v);
            head = heading(info.ex, info.ey, info.eh, anchor);
            node = struct();
            node.info = info;
            node.traj = traj;
            if dista > 0  
                score_tbl = [score_tbl; [v, w, dista, velocity, head]];
                node_tbl = [node_tbl; node];
            end
        end
    end

    % 根据配置使用相应的评价函数
    switch (cfg.score_type)
        case 'normal'

            if sum(score_tbl(:, DIST)) ~= 0
                score_tbl(:,DIST) = score_tbl(:,DIST)/sum(score_tbl(:,DIST));

            end
            if sum(score_tbl(:,VELOCITY)) ~= 0
                score_tbl(:,VELOCITY) = score_tbl(:,VELOCITY)/sum(score_tbl(:,VELOCITY));
            end
            if sum(score_tbl(:,HEADING)) ~= 0
                score_tbl(:,HEADING) = exp(score_tbl(:,HEADING))/sum(exp(score_tbl(:,HEADING)));
            end
            %? 评估函数，所以多目标规划可否使用模糊评价函数
            score_tbl(:, SCORE) = cfg.a.*score_tbl(:,HEADING) + cfg.c.*score_tbl(:,VELOCITY) + cfg.b.*score_tbl(:,DIST);

        case 'fuzzy'
            [num, ~] = size(score_tbl);
            score_tbl(:,SCORE) = zeros([num, 1]);
            for i=1:num
                theta = 2*pi - score_tbl(i, HEADING);
                dista = score_tbl(i, DIST);
                v = score_tbl(i, VELOCITY);
                score_tbl(i, SCORE) = dwa_fuzzy(theta, dista, v);
            end
    end

    id = isnan(score_tbl(:, SCORE)); % 去除nan
    score_tbl(id, :) = [];
    node_tbl(id, :) = [];
    score_tbl = array2table(score_tbl, "VariableNames", {'v', 'w', 'dist', 'velocity', 'heading', 'score'});
    node_tbl = struct2table(node_tbl );
    tbl = [score_tbl, node_tbl];
end

function score = dwa_fuzzy(theta, d_min, v)
% 模糊评价轨迹函数
    v = abs(v);
    r_h = [smf(theta, [pi/2, pi]); trimf(theta, [0, pi/2, pi]); zmf(theta, [0, pi/2])];
    
    r_d = [zmf(d_min, [0, 2]); trimf(d_min, [1,2,3]); smf(d_min, [3,5])];
    
    r_v = [zmf(v, [0, 0.5]); trimf(v, [0,0.5,1]); smf(v, [0.5,1])];
    
    R = [r_h, r_d, r_v]; % 评价矩阵
    
    A = [1;3;1.5]; % 各个因素所占权重
    
    result = R*A;
    result = result/sum(result); % 归一化，用于后续打分
    
    % 去模糊化
    w = [30, 60, 90]; % 打分，分别对应{差，中，好}
    score = w*result;
end
