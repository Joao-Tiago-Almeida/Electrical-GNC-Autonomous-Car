function ISTBreakups(file_path)
    if exist(string(folder_path + "IST_BreakUps.mat"),'file')    
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
        %TODO: Load roads and roadmarks to fig
        load(string(folder_path + "IST_BreakUps.mat"), 'ISTBreakupPoints');
        for idx=ISTBreakupPoints
            h = drawpolygon('Color','g','InteractionsAllowed','none', 'Position', cell2mat(idx));
%             h.Color = [3 0 0] / 255;
            h.FaceAlpha = 0.2;
        end
        savefig(MAP, string(folder_path + "MAP.fig"), 'compact');
    end
end