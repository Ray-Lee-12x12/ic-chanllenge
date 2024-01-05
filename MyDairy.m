classdef MyDairy
%% 用于记录的日志类
    properties
        table
    end

    methods
        function self = MyDairy()
            self.table = containers.Map;
        end

        function v = read(self, str)
            if ~isKey(self.table, str)
                v = [];
            else
                v = self.table(str);
            end
        end

        function self = write(self, var)
            self.table(inputname(2)) = var;
        end

    end
end