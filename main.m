clear all
clc

import Drone.*
import Artva.*
import Plotter.*

% Defining constants
NONE = -1;
drones_num = 2;
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
control_time = 1;
control_steps = int16(control_time/time_step);
history_est_artva = zeros(3, control_steps);
check = ones(1, control_steps);
k = 1;
threshold = 0.00001; % 0.001 m --> 1mm

% Global vars for switching trajectories can be "circ","patrol","rect"
global trajectory_type;
trajectory_type = "rect";

% Init
[drones_list, artva, est_artva] = setup(drones_num);

if(show_simulation)
    p = Plotter();
end


while true

    drones_list = replan(drones_list, drones_num, est_artva.position);

    for i = 1:drones_num
        drones_list{i} = drones_list{i}.move();
        drones_x_array(i) = drones_list{i}.position(1);
        drones_y_array(i) = drones_list{i}.position(2);
        [phi, signal] = artva.getSignal(drones_list{i}.position);
        est_H(:,i) = phi;
        est_Y(i) = signal;
    end

    est_X = est_X + inv(est_S)*est_H*(est_Y - est_H.'*est_X);
    est_S = est_beta*est_S + est_H * est_H.';
    est_artva.position = [est_X(7), est_X(8), est_X(9)];

    % Save the values to check when the algorithm is not updating the values anymore
    history_est_artva(:,k) =     [est_X(7); 
                                  est_X(8); 
                                  est_X(9)];
    last_estimate = history_est_artva(:, control_steps);
    
    if k > 1
        if norm(history_est_artva(:,k) - history_est_artva(:,k-1)) < threshold
            check(k) = 0;
        end
    end

    if k == 1
        if norm(history_est_artva(:,k) - last_estimate) < threshold
            check(k) = 0;
        end
    end
    
    % If for consecutives times the check is always true then stop the simulation
    if sum(check) == 0 && (norm(history_est_artva(:,1) - history_est_artva(:,control_steps)) < threshold)
        disp("You have estimated the goal with an accuracy of: " + threshold*100 + " m");
        break
    end

    if k >= control_steps
        k = mod(k,control_steps);
        history_est_artva = zeros(3,control_steps);
     end

    if drones_list{1}.position(2) > 1
        break;
    end

    if(show_simulation)
        p.draw([drones_x_array; drones_y_array], artva.position, est_artva.position);
        pause(time_step)
    end  
    time_instant = time_instant + time_step;
    k = k + 1;
end

p.close();

% Useful functions
function [drones_list, artva, est_artva] =  setup(drones_num)
    disp("Setup started!")
    global trajectory_type; 
    drones_list = cell([1, drones_num]);

    if trajectory_type == "rect" || trajectory_type == "patrol"
        for i = 1:drones_num
            drones_list{i} = Drone(i, [0, 0, 0]);
            drones_list{i} = drones_list{i}.setGoal([(i-1/2)/drones_num, 0, 0]);
        end
        artva = Artva([rand, rand, 0]);
    
    elseif trajectory_type == "circ"
        global angles;
        angles = zeros(1, drones_num);
        omega = 360/drones_num;
        for i = 1:drones_num
            drones_list{i} = Drone(i, [0, 0, 0]);
            omega_i = omega * (i-1);
            angles(i) = omega_i;
            m = tan(deg2rad(omega_i));
            if (omega_i > 315 && omega_i <= 360) || omega_i <= 45
                drones_list{i} = drones_list{i}.setGoal([1, m*1, 0]);
            elseif omega_i > 45 && omega_i <= 135
                drones_list{i} = drones_list{i}.setGoal([1/m, 1 0]);
            elseif omega_i > 135 && omega_i <= 225
                drones_list{i} = drones_list{i}.setGoal([-1, m*-1, 0]);
            elseif omega_i > 225 && omega_i <= 315
                drones_list{i} = drones_list{i}.setGoal([-1/m, -1, 0]);
            end 
        end
        artva = Artva([-1 + 2 * rand,-1 + 2 * rand,0]); %random value between -1 and 1
    end 

    est_artva = Artva([0.0, 0.0, 0]);


    disp("Setup completed!")
end

% New trajectories (circle, border patrol)
function new_drones_list = replan(drones_list, drones_num, est_artva_pos)
    global trajectory_type
    if trajectory_type == "rect"
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

    elseif trajectory_type == "circ"
        all_idle = true;

        for i = 1:drones_num
            if drones_list{i}.state ~= "idle" || ~drones_list{i}.isAtGoal()
                all_idle = false;
            end
        end

        if all_idle
            disp("Replanning!");
            new_drones_list = cell([1, drones_num]);
            for i = 1:drones_num
                current_drone = drones_list{i};
                current_drone = current_drone.setGoal(est_artva_pos);
                new_drones_list{i} = current_drone;
            end
        else
            new_drones_list = drones_list;  
        end
        return

    elseif trajectory_type == "patrol"
        % VALE QUESTO PER TE 
        disp("Patrol trajectory not implemented yet!");
        return

    else
        error("Unknown trajectory type: %s", trajectory);
    end
end

