%%
function route = Planning(global_map, sp, ep)
% 规划全局路径
    route = astar(global_map, sp, ep);
end

%%
function route = astar(global_map, sp, ep)
% astar算法的实现
    % global_map tenor(50,50) sp struct(x,y) ep struct(x,y)
    X = 1;
    Y = 2;
    G = 3;
    H = 4;
    F = 5;
    PX = 6;
    PY = 7;
    sz = [50, 50]; % 地图的尺寸
    sp = [sp.x, sp.y];
    ep = [ep.x, ep.y];
    h_func = @(p, ep)(sqrt((p(X) - ep(X))^2 + (p(Y) - ep(Y))^2)); % 启发函数
    open = []; % table(x,y,g,h,f,parent,px,py)
    close = []; % table(x,y,g,h,f,parent,px,py)
    success = false; % bool
    motion = [1, 1; 1, 0; 1, -1; 0, 1; 0, -1; -1, 1; -1, 0; -1, -1]; % 八领域搜索

    route = []; % table(x,y,g,h,f,paren,px,py)

    n_sp = [sp(X), sp(Y), 0, h_func(sp, ep), h_func(sp, ep), nan, nan];
    open = [open; n_sp];

    while ~isempty(open)
        id = open(:, F) == min(open(:, F)); % mask
        n = open(id, :);
        if size(n) > 1
            n = n(1,:);
        end
        close = [close; n];
        open(id, :) = [];
       

        if ep(X) == n(X) && ep(Y) == n(Y)
            success = true;
            break;
        end
        for m=1:length(motion)
            delta = norm(motion(m, :));
            p = n;
            p(X) = p(X) + motion(m, 1);
            p(Y) = p(Y) + motion(m, 2);
            % 相当于生成子节点
            if p(X)<1||p(X)>=50||p(Y)<1||p(Y)>=50
                continue;
            end
            if global_map(p(X), p(Y)) ~= 0
                continue;
            end
    
            %%% 1.在close表
            if ~isempty(close(close(:, X) == p(X) & close(:,Y) == p(Y), :))
                continue;
            end
            %%% 2.在open表
            if ~isempty(open(open(:, X) == p(X) & open(:,Y) == p(Y), :))
                id = open(:, X) == p(X) & open(:,Y) == p(Y);
                h = h_func(p, ep);
                if h + delta + n(G) < p(F)
                    open(id, G) = n(G) + delta;
                    open(id, F) = h + delta + n(G);
                    open(id, PX) = n(X);
                    open(id, PY) = n(Y);
                end
            %%% 3.不在open表
            else
                p(G) = n(G) + delta;
                p(H) = h_func(p, ep);
                p(F) = p(G) + p(H);
                p(PX) = n(X);
                p(PY) = n(Y);
                open = [open; p];
            end
        end
    end
    %%% 寻路成功，根据parent生成路径表
    if success
        t = close(close(:, X) == ep(X) & close(:, Y) == ep(Y), :);
        while ~isnan(t(PX)) && ~isnan(t(PY))
            route = [t; route];
            new_x = t(PX);
            new_y = t(PY);
            t = close(close(:, X) == new_x & close(:, Y) == new_y, :);
        end
        route = [t; route];
    end
    
    if isempty(route)
        pause(60);
    end


    route = array2table(route, "VariableNames", {'x', 'y', 'g', 'h', 'f', 'px', 'py'});
end
% 
% function h_table = h_func_table(ep_vec, map)
%     [Y, X] = meshgrid(1:50);
%     h_table = abs(X - ep_vec(1)) + abs(Y - ep_vec(2));
% end

    