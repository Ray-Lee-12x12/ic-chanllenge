classdef Policy < handle
    properties
        map
        log
        route
        feer
        score_type
    end

    methods
        function self = Policy()
            self.map = zeros([50, 50]); % 地图
            self.log = MyDairy(); % 日志
            self.route = []; % 路径
            self.feer = 0; % 恐惧值

            self.score_type = 'normal'; % 评价函数类型
        end

        function action=action(self, observation)
            % disp(observation.t); 采样周期和决策周期为0.3s
            s_pos = observation.startPos;
            e_pos = observation.endPos;
            cur_pos = struct('x', observation.agent.x, ...
                'y', observation.agent.y);
            cur_pos_int = struct('x', double(int32(observation.agent.x)), ...
                'y', double(int32(observation.agent.y)));
            scan_car_map = observation.scanMap;
            sz = [50, 50];
            out_of_area = false;

            %% perception
            scan_car_map = pretreatScanMap(scan_car_map);
            cur_map = self.map | (scan_car_map==1);
            %% global plan
            self.map = cur_map;
            % 地图膨胀
            dilate_map = imdilate(self.map, strel("disk", 1));
            % 防止过度膨胀将终点掩盖
            for i=-1:1
                for j=-1:1
                    if(self.map(e_pos.x + i, e_pos.y + j) == 0)
                        dilate_map(e_pos.x + i, e_pos.y + j) = 0;
                    end
                end
            end
            
            % 一般来说这个是多余的，但是在打开smart agent的时候有可能遇到
            if cur_pos_int.x < 1
                cur_pos_int.x = 1; out_of_area = true;
            elseif cur_pos_int.x >50
                cur_pos_int.x = 50; out_of_area = true;
            end

            if cur_pos_int.y < 1
                cur_pos_int.y = 1; out_of_area = true;
            elseif cur_pos_int.y >50
                cur_pos_int.y = 50; out_of_area = true;
            end
            self.route = Planning(dilate_map, cur_pos_int, e_pos);
            % self.route = Planning(dilate_map, s_pos, e_pos);

            %%% 记录
            self.log...
                .write(scan_car_map)...
                .write(dilate_map);

            if isempty(self.route)
                disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
                disp("empty");% 无可达路径
                disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
            end
            %% local plan
            % 圆(x,y,3)可以大致将小车包裹
            if size(self.route) > 3
                dist = sqrt((self.route.x - cur_pos.x).^2 + (self.route.y - cur_pos.y).^2);
                id = find(dist < 5);
                if ~out_of_area
                    if length(id) > 1
                        id = id(end);
                    end
                    next_vec = self.route{id,{'x','y'}};
                    next_vec = [next_vec(1); next_vec(2)];
                else
                    next_vec = [cur_pos_int.x, cur_pos_int.y];
                end
            else
                % id = size(self.route);
                next_vec = [e_pos.x; e_pos.y];
            end
            %% control
            if ~isempty(find(observation.scanMap ~= 0, 1))
                % 恐惧值是针对前方障碍物的，当前方有障碍物时增加恐惧值
                self.feer = self.feer * 0.8 + observation.collide;
            else
                self.feer = self.feer*0.8;
            end
            id_car = scan_car_map ~= 0 & scan_car_map ~= 1;
            scan_car_map = zeros([50, 50]);
            scan_car_map_linear = scan_car_map(:);
            scan_car_map_linear(id_car) = 1;
            scan_car_map = reshape(scan_car_map_linear, sz);
            dilate_car_map = imdilate(scan_car_map, strel("sphere", 1));

            action = Motion(dilate_map | dilate_car_map, observation.agent,...
                struct('x', next_vec(1), 'y', next_vec(2)),...
                self.feer, self.score_type);
            %% log
            %%% 记录
            self.log...
                .write(cur_pos)...
                .write(next_vec);
            self.view(); % 可视化
        end

        function view(self)
            % 绘制辅助图
            figure(1);
            persistent hdl1;
            persistent hdl2;
            persistent hdl3;
            persistent hdl4;
            % 绘制规划路线
            if ~isempty(self.route)
                if isempty(hdl1)
                    hdl1 = plot(self.route{:,{'x'}}, self.route{:,{'y'}});
                else
                    set(hdl1, 'XData', self.route{:,{'x'}}, 'YData', self.route{:,{'y'}})
                end
            end
            % 绘制膨胀地图
            dilate_map = self.log.read("dilate_map");
            if ~isempty(dilate_map)
                [r, c] = find(dilate_map ~= 0);
                if isempty(hdl2)
                    hdl2 = scatter(r, c, 'Marker', 'x');
                else
                    set(hdl2, 'XData', r, 'YData', c);
                end
            end
            % 绘制当前点
            cur_pos = self.log.read("cur_pos");
            cur_vec = [cur_pos.x; cur_pos.y];
            next_vec = self.log.read("next_vec");
            if ~isempty(cur_pos)
                if isempty(hdl3)
                    hdl3 = scatter(cur_pos.x, cur_pos.y, 'Marker', 'v');
                else
                    if isempty(next_vec)
                        scatter(cur_vec(1), cur_vec(2));
                    else
                        set(hdl3, 'XData', [next_vec(1), cur_vec(1)], 'YData', [next_vec(2), cur_vec(2)]);
                    end
                end
            end
            % 绘制当前地图
            [r, c] = find(self.map ~= 0);
            if isempty(hdl4)
                hdl4 = scatter(r, c, 'Marker', 'square');
            else
                set(hdl4, 'XData', r, 'YData', c);
            end

            persistent scan_map_list;
            scan_map = self.log.read("scan_map");
            scan_map_list = [scan_map_list; scan_map(:)];
%             figure(12);
%             imshow(imrotate(imresize(self.log.read("scan_map"), 4,"nearest"), 90));
% 
%             figure(13);
%             imshow(imrotate(imresize(self.map, 4,"nearest"), 90));
% 
%             figure(15);
%             load ./maps/a1map.mat;
%             [r, c] = find(imdilate(mapdata, strel("diamond", 1))); 
%             scatter(r, c, "Marker", '+');
%             hold on;
%             [r, c] = find(mapdata); 
%             scatter(r,c); waitforbuttonpress;
%             pause(0.1);
        end
    end
end


function scan_map_after = pretreatScanMap(scan_map)
    % 由于scan_map和occupy_map存在偏差，为弥补偏差做一些预处理
    sz = [50, 50];
    [r, c] = find(scan_map == 1);
    scan_map_linear = scan_map(:);
    tmp = table();
    tmp.r = r - 1;
    tmp.c = c;
    tmp(tmp.r < 1 | tmp.r > 50, :) = [];
    id1 = sub2ind(sz, tmp.r, tmp.c);
    tmp = table();
    tmp.r = r;
    tmp.c = c - 1;
    tmp(tmp.c < 1 | tmp.c > 50, :) = [];
    id2 = sub2ind(sz, tmp.r, tmp.c);
    tmp = table();
    tmp.r = r - 1;
    tmp.c = c - 1;
    tmp(tmp.r < 1 | tmp.r > 50 | tmp.c < 1 | tmp.c > 50, :) = [];
    id3 = sub2ind(sz, tmp.r, tmp.c);

    scan_map_linear(id1) = 1;
    scan_map_linear(id2) = 1;
    scan_map_linear(id3) = 1;
    scan_map_after = reshape(scan_map_linear, sz);
end