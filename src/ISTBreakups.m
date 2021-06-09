function occupancy_matrix = ISTBreakups(folder_path, path_img, occupancy_matrix)
    if exist(string(folder_path + "IST_BreakUps.mat"),'file')
        [X,Y] = meshgrid(1:size(occupancy_matrix, 2),1:size(occupancy_matrix, 1));
        %load(string(file_path+"MAP.mat"),'MAP');
        %MAP = openfig(string(file_path+"MAP.fig"));
        MAP = figure('Name','MAP','NumberTitle','off');
        pbaspect([1 1 1]);
        I = imread(path_img);
        MAP.Children.Position = [0 0 1 1];
        imshow(I);
        MAP.WindowStyle = 'docked';
        MAP.Units = 'pixel';
        hold on;
        load(string(folder_path + "roads.mat"), 'roads');
        for idx=roads
            h = drawpolygon('Color','k','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = [130 130 130] / 255;
            h.FaceAlpha = 0.2;
        end
        
        load(string(folder_path + "cross_walks.mat"), 'cross_walks');
        load(string(folder_path + "traffic_lights.mat"), 'traffic_lights');
        load(string(folder_path + "stop_signs.mat"), 'stop_signs');
        load(string(folder_path + "speed_limits.mat"), 'speed_limits');
        for idx=cross_walks
            h = drawpolygon('Color','w','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = 'white';
            h.FaceAlpha = 0.1;
        end
        for idx=traffic_lights
            h = drawpolygon('Color','g','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = 'green';
            h.FaceAlpha = 0.1;
         end
        for idx=stop_signs
            h = drawpolygon('Color','r','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = 'red';
            h.FaceAlpha = 0.1;
        end
        for idx=speed_limits
            h = drawpolygon('Color','c','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = 'cyan';
            h.FaceAlpha = 0.1;
        end
        %TODO: Load roads and roadmarks to fig
        load(string(folder_path + "IST_BreakUps.mat"), 'ISTBreakupPoints');
        for idx=ISTBreakupPoints
            h = drawpolygon('Color','y','InteractionsAllowed','none', 'Position', cell2mat(idx));
            h.Color = [255 204 102] / 255;
            h.FaceAlpha = 0.2;
            gps_breakup = cell2mat(idx);
            occupancy_matrix(logical(inpolygon(X, Y, gps_breakup(:,1)', gps_breakup(:,2)' ) .* occupancy_matrix)) = 6;
        end
        savefig(MAP, string(folder_path + "MAP.fig"), 'compact');
    end
end