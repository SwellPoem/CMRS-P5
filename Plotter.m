classdef Plotter < handle
    properties
        fig
        is_initialized
        scatter_drones
        scatter_artva
        scatter_est_artva
        ax
    end
    methods
        % Constructor
        function obj = Plotter()
            obj.fig = figure;
            obj.is_initialized = false;
            obj.scatter_drones = -1;
            obj.scatter_artva = -1;
            obj.scatter_est_artva = -1;
            obj.ax = -1;
        end

        function obj = draw(obj, drones_pos, artva_pos, est_artva_pos)
            global trajectory_type;
            if(~obj.is_initialized)
                obj.scatter_artva = scatter(artva_pos(1), artva_pos(2), '*', 'red');
                hold on
                obj.scatter_est_artva = scatter(est_artva_pos(1), est_artva_pos(2), '*', 'green');
                obj.scatter_drones = scatter(drones_pos(1,:), drones_pos(2,:), '*', 'blue');
                obj.ax = gca; % gca is Matlab's way of getting the current axes
                if trajectory_type == "rect" || trajectory_type == "patrol"
                    obj.ax.XLim = [0 1];
                    obj.ax.YLim = [0 1];
                elseif trajectory_type == "circ"
                    global angles;
                    obj.ax.XLim = [-1 1];
                    obj.ax.YLim = [-1 1];
                    viscircles([0, 0], 1, 'LineStyle', '--', 'EdgeColor', 'b');
                    for i = 1:length(angles)
                        angle = deg2rad(angles(i));
                        line([0, cos(angle)], [0, sin(angle)],'LineStyle','--','Color','r');
                    end     
                end 
                obj.is_initialized = true;
                hold off
            else
                obj.scatter_drones.XData = drones_pos(1,:);
                obj.scatter_drones.YData = drones_pos(2,:);
                obj.scatter_artva.XData = artva_pos(1);
                obj.scatter_artva.YData = artva_pos(2);
                obj.scatter_est_artva.XData = est_artva_pos(1);
                obj.scatter_est_artva.YData = est_artva_pos(2);
            end
        end

        function obj = close(obj)
            close(obj.fig);
        end
    end
end