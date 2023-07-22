clear all
clc

import Drone.*
import Artva.*
import Plotter.*

% Defining constants
NONE = -1;
drones_num = 10;
drones_list = NONE;
drones_x_array = zeros(1, drones_num);
drones_y_array = zeros(1, drones_num);
artva = NONE;
est_artva = NONE;
est_S = eye(10);
est_beta = 1.0;
est_Y = zeros(drones_num, 1);
est_X = zeros(10, 1);
est_H = zeros(10, drones_num);
p = NONE;
time_instant = 0;
time_step = 0.01;
show_simulation = true;

% Init
[drones_list, artva, est_artva] = setup(drones_num);

if(show_simulation)
    p = Plotter();
end

while true

    drones_list = replan(drones_list, drones_num);

    for i = 1:drones_num
%         drones_list{i}.position = drones_list{i}.position + time_step*[cos(pi*(i-1)/(2*(drones_num-1))), sin(pi*(i-1)/(2*(drones_num-1))), 0];
        drones_list{i} = drones_list{i}.move();
        drones_x_array(i) = drones_list{i}.position(1);
        drones_y_array(i) = drones_list{i}.position(2);
        [phi, signal] = artva.getSignal(drones_list{i}.position);
        est_H(:,i) = phi;
        est_Y(i) = signal;
    end

    est_X = est_X + inv(est_S)*est_H*(est_Y - est_H.'*est_X);
    %est_X(isnan(est_X))=0;
    est_S = est_beta*est_S + est_H * est_H.';
    est_artva.position = [est_X(7), est_X(8), est_X(9)];

    if drones_list{1}.position(2) > 1
        break;
    end

    if(show_simulation)
        p.draw([drones_x_array; drones_y_array], artva.position, est_artva.position);
        pause(time_step)
    end
    time_instant = time_instant + time_step;
end

p.close();

% Useful functions
function [drones_list, artva, est_artva] =  setup(drones_num)
    disp("Setup started!")
    
    drones_list = cell([1, drones_num]);
    for i = 1:drones_num
        drones_list{i} = Drone(i, [0, 0, 0]);
        drones_list{i} = drones_list{i}.setGoal([(i-1/2)/drones_num, 0, 0]);
    end

    artva = Artva([rand, rand, 0]);
    est_artva = Artva([0.5, 0.5, 0]);

    disp("Setup completed!")
end

function new_drones_list = replan(drones_list, drones_num)
    for i = 1:drones_num 
        if(drones_list{i}.state ~= "idle" || ~drones_list{i}.isAtGoal())
            new_drones_list = drones_list;
            return
        end
    end
    disp("Replanning!")
    new_drones_list = cell([1, drones_num]);
    for i = 1:drones_num
        current_drone = drones_list{i};
        g = current_drone.goal;
        if(g(2) ~= 0)
            current_drone = current_drone.setGoal([g(1), 0.0, g(3)]);
        else
            current_drone = current_drone.setGoal([g(1), 1.0, g(3)]);
        end
        new_drones_list{i} = current_drone;
        clear current_drone g;
    end
end
