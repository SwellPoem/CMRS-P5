classdef Plotter < handle
    properties
        fig
        is_initialized
        scatter_drones
        scatter_artva
        scatter_est_artva
        ax
        est_labels_handles % New property to store the handles of estimate text labels
        drone_labels_handles % New property to store the handles of drone text labels
        averages_labels_handles
        is_text_updated % New property to track if text is updated
        scatter_est_mean % mean of all the estimates
        scatter_consensus_mean % mean calculated with consensus by each drone
        th % title
    end
    methods
        % Constructor
        function obj = Plotter()
            obj.fig = figure;
            obj.is_initialized = false;
            obj.scatter_drones = -1;
            obj.scatter_artva = -1;
            obj.scatter_est_artva = -1;
            obj.scatter_est_mean = -1;
            obj.scatter_consensus_mean = -1;
            obj.ax = -1;
            obj.est_labels_handles = gobjects(1, 0); % Initialize as an empty array
            obj.drone_labels_handles = gobjects(1, 0); % Initialize as an empty array
            obj.averages_labels_handles = gobjects(1, 0);
            obj.is_text_updated = false; % Initialize to false
            obj.th = title('Title');
        end

        function obj = draw(obj, drones_pos, artva_pos, est_artva_pos,time_instant,consensus_mean_array)
            global trajectory_type;
            global distributed_estimation_mode;
            global see_text;
            global angles;
            n_drones = size(drones_pos, 2);
            dx = 0.01;
            
            if (~obj.is_initialized)
                % Generate colors for plot
                if distributed_estimation_mode == false
                    C = linspecer(3);
                    obj.scatter_artva = scatter(artva_pos(1), artva_pos(2), 100, C(2, :), '*');
                    hold on
                    obj.scatter_est_artva = scatter(est_artva_pos(1), est_artva_pos(2), 100, C(3, :), '*');
                    obj.scatter_drones = scatter(drones_pos(1, :), drones_pos(2, :), 100, C(1, :), 'o', 'filled');
                    % Legend
                    %legend('true position', 'estimated position', 'drones position', 'Location', 'NorthEast','AutoUpdate','off');

                elseif distributed_estimation_mode == true
                    % Plot utilities
                    C = linspecer(n_drones*2, 'sequential');
                    c1 = C(1:2:end, :); % Rows 1, 3,...
                    c2 = C(2:2:end, :); % Rows 2, 4,...

                    est_mean = mean(est_artva_pos,2);
                    obj.scatter_est_mean = scatter(est_mean(1),est_mean(2),100,'*','black');
                    if see_text == true
                        for i=1:n_drones
                            obj.est_labels_handles(i) = text(obj.ax, est_artva_pos(1,i)+dx, est_artva_pos(2,i), num2str(i),'FontSize', 10);
                            obj.drone_labels_handles(i) = text(obj.ax, drones_pos(1,i)+dx, drones_pos(2,i), num2str(i),'FontSize', 10);
                            %obj.averages_labels_handles(i) = text(obj.ax, consensus_mean_array(1,i)+dx, consensus_mean_array(2,i), num2str(i),'FontSize', 10);
                        end
                    end
                    obj.scatter_artva = scatter(artva_pos(1), artva_pos(2), 100,'*','red','DisplayName','true position');
                    hold on
                    obj.scatter_est_artva = scatter(est_artva_pos(1,:), est_artva_pos(2,:), 100, c1, '*','HandleVisibility', 'off');
                    obj.scatter_drones = scatter(drones_pos(1, :), drones_pos(2, :), 100, c2, 'o', 'filled','HandleVisibility', 'off');
                    est_mean = mean(est_artva_pos,2);
                    obj.scatter_est_mean = scatter(est_mean(1),est_mean(2),100,'*','black');
                    % TODO cambiare colori (maybe) e mettere che magari le stime non si vedono 
                    obj.scatter_consensus_mean = scatter(consensus_mean_array(1,:),consensus_mean_array(2,:),100,c2,'diamond','filled');
                    % Legend
                    %legend('Location','best','AutoUpdate','off');
                end
                obj.ax = gca; % gca is Matlab's way of getting the current axes

                if trajectory_type == "rect"
                    obj.ax.XLim = [0 1];
                    obj.ax.YLim = [0 1];
                    for i = 1:n_drones
                        line([(i-1/2)/n_drones, (i-1/2)/n_drones], [0, 1], 'LineStyle', '--', 'Color', 'black', 'HandleVisibility', 'off')
                    end
                elseif trajectory_type == "circ"
                    obj.ax.XLim = [-1 1];
                    obj.ax.YLim = [-1 1];
                    viscircles([0, 0], 1, 'LineStyle', '--', 'EdgeColor', 'black');
                    for i = 1:length(angles)
                        angle = deg2rad(angles(i));
                        line([0, cos(angle)], [0, sin(angle)], 'LineStyle', '--', 'Color', 'black', 'HandleVisibility', 'off');
                    end     
                elseif trajectory_type == "patrol"
                    % draw a circle centered on the bottom left corner and with radius 1
                    obj.ax.XLim = [0 1];
                    obj.ax.YLim = [0 1];
                    viscircles([0, 0], 1, 'LineStyle', '--', 'EdgeColor', 'black');
                    for i = 1:length(angles)
                        angle = deg2rad(angles(i));
                        line([0, cos(angle)], [0, sin(angle)],'LineStyle','--','Color','black','HandleVisibility', 'off');
                    end
                end
                obj.is_initialized = true;
                hold off
            else
                titleString = sprintf('Simulation Time: %0.2f sec', time_instant);
                obj.th = title(obj.ax, titleString);
                obj.scatter_drones.XData = drones_pos(1,:);
                obj.scatter_drones.YData = drones_pos(2,:);
                obj.scatter_artva.XData = artva_pos(1);
                obj.scatter_artva.YData = artva_pos(2);
                if(size(est_artva_pos, 1) == 2)
                    obj.scatter_est_artva.XData = est_artva_pos(1,:);
                    obj.scatter_est_artva.YData = est_artva_pos(2,:);
                    est_mean = mean(est_artva_pos,2);
                    obj.scatter_est_mean.XData = est_mean(1);
                    obj.scatter_est_mean.YData = est_mean(2);
                    obj.scatter_consensus_mean.XData = consensus_mean_array(1,:);
                    obj.scatter_consensus_mean.YData = consensus_mean_array(2,:);
                elseif(size(est_artva_pos, 1) == 1)
                    obj.scatter_est_artva.XData = est_artva_pos(1);
                    obj.scatter_est_artva.YData = est_artva_pos(2);
                    % Nel caso in cui non-distribuito the mean does not exist
                end
                %%% RICORDATI CHE DEVI SCOMMENTARE ANCHE LA PARTE SOPRA SE VUOI IL TESTO ANCHE PER LE MEDIE
                    if see_text == true && distributed_estimation_mode == true
                        % Add the text to the drones, nell altro caso non serve 
                        for i=1:n_drones
                            % Delete previous text labels
                            delete(obj.est_labels_handles(i));
                            delete(obj.drone_labels_handles(i));
                            %delete(obj.averages_labels_handles(i));
                            obj.est_labels_handles(i) = text(obj.ax, est_artva_pos(1,i)+dx, est_artva_pos(2,i), num2str(i),'FontSize', 10);
                            obj.drone_labels_handles(i) = text(obj.ax, drones_pos(1,i)+dx, drones_pos(2,i), num2str(i),'FontSize', 10);
                            %obj.averages_labels_handles(i) = text(obj.ax, consensus_mean_array(1,i)+dx, consensus_mean_array(2,i), num2str(i),'FontSize', 10);
                        end
                        % Set is_text_updated to true to indicate that text is updated
                        obj.is_text_updated = true;
                    end 
            end

        end

        function obj = close(obj)
            close(obj.fig);
        end
    end
end
