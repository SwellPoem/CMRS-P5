clear all;
close all;  % Close all open figure windows
clc

import Drone.*
import Artva.*
import Plotter.*

%% CHOSEN Variables
show_simulation = true;
global threshold;
%threshold = 0.00001; % 0.001 m --> 1mm
%threshold = 0.0001; % 0.01 m --> 1cm
%threshold = 0.001; % 0.1 m --> 10 cm
%threshold = 0.005; % 0.5 m --> 50 cm
threshold = 0.0060; % 0.6 m --> 60 cm
global control_time;
control_time = 2;
global time_step;
time_step = 0.01;
global distributed_estimation_mode;
distributed_estimation_mode = true;
global dronesSetted;
dronesSetted = false;
global trajectory_type;

trajectory_type = "rect"; % Either "circ","patrol","rect"

%% Constants
NONE = -1;
global drones_num;
drones_num = 6;
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
global control_steps;
control_steps = int16(control_time/time_step);
global history_est_artva;
history_est_artva = zeros(3*drones_num, control_steps);
check = ones(drones_num, control_steps);
global majority;
majority = 0;
k = 1;

% Variables specific to Distributed mode
est_artva_x_array = zeros(1, drones_num);
est_artva_y_array = zeros(1, drones_num);
consensus_mean_array = zeros(2,drones_num);
sync_delay = 1; % In seconds


%% Init
[drones_list, artva, est_artva] = setup(drones_num);

if(show_simulation)
    p = Plotter();
end

%% Simulation loop
while true

    if(~distributed_estimation_mode)

        drones_list = replan(drones_list, drones_num, est_artva.position);

        for i = 1:drones_num
            drones_list{i} = drones_list{i}.move();
            drones_x_array(i) = drones_list{i}.position(1);
            drones_y_array(i) = drones_list{i}.position(2);
            [phi, signal] = artva.getSignal(drones_list{i}.position);
            est_H(:,i) = phi;
            est_Y(i) = signal;
        end

        %       est_X = est_X + inv(est_S)*est_H*(est_Y - est_H.'*est_X);
        est_X = est_X + est_S\(est_H*(est_Y - est_H.'*est_X)); % Should be better than previous version
        est_S = est_beta*est_S + est_H * est_H.';
        est_artva.position = [est_X(7), est_X(8), est_X(9)];

        [result,check,k] = stopping_criterium([est_X(7), est_X(8), est_X(9)]',check,k);

        if result
            break;
        end

    else

        drones_list = replan(drones_list, drones_num, [est_artva_x_array; est_artva_y_array; zeros(1,drones_num)]);

        for i = 1:drones_num

            drones_list{i} = drones_list{i}.move();
            drones_x_array(i) = drones_list{i}.position(1);
            drones_y_array(i) = drones_list{i}.position(2);

            if(mod(time_instant, sync_delay) <= 0.01)
                drones_list{i} = drones_list{i}.sync(drones_list);
                if(i==1)
                    disp("Syncing")
                end
            end

            drones_list{i} = drones_list{i}.estimate(artva);
            est_artva_x_array(i) = drones_list{i}.est_pos(1);
            est_artva_y_array(i) = drones_list{i}.est_pos(2);
            
            % Ace filter
            drones_list{i} = drones_list{i}.ace(drones_list);
            consensus_mean_array(:,i) = drones_list{i}.z_new;

        end

        [result,check,k] = stopping_criterium([est_artva_x_array;est_artva_y_array;zeros(1,drones_num)],check,k);

        if result
            break;
        end

    end

    if drones_list{1}.position(2) > 1
        break;
    end

    if(show_simulation)
        if(~distributed_estimation_mode)
            p.draw([drones_x_array; drones_y_array], artva.position, est_artva.position);
        else
            p.draw([drones_x_array; drones_y_array], artva.position, [est_artva_x_array; est_artva_y_array],consensus_mean_array);
        end
        pause(time_step)
    end
    time_instant = time_instant + time_step;
    k = k + 1;
end

p.close();

%% Functions

function [result,check,k] = stopping_criterium(est_arva_pos,check,k)
result = false;
global distributed_estimation_mode;
global threshold;
global history_est_artva;
global control_steps;
global drones_num;
global majority;
global control_time;
global dronesSetted;
global trajectory_type;


if(~distributed_estimation_mode)
    % Save the values to check when the algorithm is not updating the values anymore
    history_est_artva(1:3,k) =  est_arva_pos;
    last_estimate = history_est_artva(1:3, control_steps);

    if k > 1
        if norm(history_est_artva(1:3,k) - history_est_artva(1:3,k-1)) < threshold
            check(1,k) = 0;
        end
    end

    if k == 1
        if norm(history_est_artva(1:3,k) - last_estimate) < threshold
            check(1,k) = 0;
        end
    end

    % If for consecutives times the check is always true then stop the simulation
    if sum(check(1,:)) == 0 && (norm(history_est_artva(1:3,1) - history_est_artva(1:3,control_steps)) < threshold)
        disp("You have estimated the goal with an accuracy of: " + threshold*100 + " m");
        result = true;
    end

    if k >= control_steps
        k = mod(k,control_steps);
        history_est_artva = zeros(3,control_steps);
        check = check(1,:);
    end

else
    % The drones stop when their estimate doesn't change with respect to
    % the mean of the estimates --> convergence

    % Save the values to check when the algorithm is not updating the values anymore
    if k >= control_steps
    history_est_artva(1:3,k) =  mean(est_arva_pos,2);
    last_estimate = history_est_artva(1:3, control_steps);
    else
        last_estimate = -1;
    end

    if (trajectory_type == "rect" && dronesSetted)
        for i= 1:drones_num
            if mean(est_arva_pos,2) - est_arva_pos(:,i) < threshold
                check(i,k) = 0;
            end
        end
    end

    if(trajectory_type == "circ" || trajectory_type == "patrol")
        for i= 1:drones_num
            if mean(est_arva_pos,2) - est_arva_pos(:,i) < threshold
                check(i,k) = 0;
            else
                check(i,k) = 1;
            end
        end
    end

    %sum(check(:))
    if sum(check(:)) == 0 || norm(history_est_artva(1:3,1) - last_estimate) < threshold
        disp("The estimate of all drones wrt the mean did not change, you have estimated the goal with an accuracy of: " + threshold*100 + " m");
        result = true;
    end

    % % Save the values to check when the algorithm is not updating the values anymore
    % mean(est_arva_pos,2)
    % history_est_artva(:,k) =  reshape(est_arva_pos, [], 1);
    % last_estimate = history_est_artva(:, control_steps);
    % 
    % for i=1:drones_num
    %     start_idx = (i - 1) * 3 + 1;
    %     end_idx = i * 3;
    %     if k > 1
    %         if norm(history_est_artva(start_idx:end_idx,k) - history_est_artva(start_idx:end_idx,k-1)) < threshold
    %             check(i,k) = 0;
    %         end
    %     end
    %     if k == 1
    %         if norm(history_est_artva(start_idx:end_idx,k) - last_estimate(start_idx:end_idx)) < threshold
    %             check(i,k) = 0;
    %         end
    %     end
    % 
    %     % CONSENSUS: if the majority agrees (not on the same values, but its value dont change then stop)
    %     if sum(check(i,:)) == 0 && (norm(history_est_artva(start_idx:end_idx,1) - history_est_artva(start_idx:end_idx,control_steps)) < threshold)
    %         majority = majority + 1;
    %         disp("Drones that have not update their estimate in less more than " + control_time + "s are: " + majority)
    %     end
    % end
    % 
    % if majority > ceil(drones_num / 2)
    %     disp("You have estimated the goal with an accuracy of: " + threshold*100 + " m");
    %     result = true;
    % end

    if k >= control_steps
        k = mod(k,control_steps);
        %history_est_artva = zeros(3*drones_num, control_steps);
        history_est_artva = zeros(3, control_steps);
    end
end
end

function [drones_list, artva, est_artva] =  setup(drones_num)
disp("Setup started!")
global trajectory_type;
drones_list = cell([1, drones_num]);

if trajectory_type == "rect"
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

elseif trajectory_type == "patrol"
    global angles;
    angles = zeros(1, drones_num);
    theta = 90/(drones_num-1);

    % initialization of all drones in the lower left corner
    for i = 1:drones_num
        drones_list{i} = Drone(i, [0, 0, 0]);
        drones_list{i} = drones_list{i}.setGoal([0, 0, 0]);
    end

    % radial movement of the central drones
    for j = 2:drones_num-1

        theta_j = theta * (j-1);
        angles(j)= theta_j;
        s = tan(deg2rad(theta_j));

        if (theta_j <= 45)
            drones_list{j} = drones_list{j}.setGoal([1, 1*s, 0]);
        
        elseif theta_j > 45
            drones_list{j} = drones_list{j}.setGoal([1/s, 1, 0]);

        end
    end

    % movement of the first drone on the x axis towards the bottom right
    % corner
    drones_list{1} = drones_list{1}.setGoal([1, 0, 0]);

    % movement of the last drone on the y axis towards the top left corner
    drones_list{drones_num} = drones_list{drones_num}.setGoal([0, 1, 0]);

    artva = Artva([rand, rand, 0]);

end

est_artva = Artva([0.0, 0.0, 0]);
disp("Setup completed!")
end

function new_drones_list = replan(drones_list, drones_num, est_artva_pos)
global trajectory_type;
global dronesSetted;
global distributed_estimation_mode;
if trajectory_type == "rect"
    for i = 1:drones_num
        if(drones_list{i}.state ~= "idle" || ~drones_list{i}.isAtGoal())
            new_drones_list = drones_list;
            return
        end
    end
    disp("Replanning!")
    dronesSetted = true;
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
            if(~distributed_estimation_mode)
                current_drone = current_drone.setGoal(est_artva_pos);
            else
                new_objective = est_artva_pos(:,i)';
                current_drone = current_drone.setGoal(new_objective);
            end
            new_drones_list{i} = current_drone;
        end
    else
        new_drones_list = drones_list;
    end
    return

elseif trajectory_type == "patrol"
    new_drones_list = drones_list;
    isAllIdle = true;
    firstIsIdle = true;
    lastIsIdle = true;
    
    % check whether central drones are idle or not
    for i = 2:drones_num-1
        if (drones_list{i}.state ~= "idle" || ~drones_list{i}.isAtGoal())
            isAllIdle = false;
        end
    end
    
    % check if first drone is at the bottom right corner (idle)
    if (drones_list{1}.state ~= "idle" || ~drones_list{1}.isAtGoal())
        firstIsIdle = false;
    end
    
    % check if last drone is at the top left corner (idle)
    if (drones_list{drones_num}.state ~= "idle" || ~drones_list{drones_num}.isAtGoal())
        lastIsIdle = false;
    end
    
    % if the first and the last drone are in position (position specified
    % before) move to the top right angle (common goal)
    if (firstIsIdle && lastIsIdle)
        disp("Replanning");
        current_drone = drones_list{1};
        current_drone = current_drone.setGoal([1,1,0]);
        new_drones_list{1} = current_drone;
        clear current_drone;

        current_drone = drones_list{drones_num};
        current_drone = current_drone.setGoal([1,1,0]);
        new_drones_list{drones_num} = current_drone;
        clear current_drone;
    end

    % if the central drones are all idle move towards the estimated position of
    % the artva
    if isAllIdle
        disp("Replanning!");
        for i =1:drones_num
            if (i ~= 1 && i ~= drones_num)
                current_drone = drones_list{i};
                if(~distributed_estimation_mode)
                    current_drone = current_drone.setGoal(est_artva_pos);
                else
                    new_objective = est_artva_pos(:,i)';
                    current_drone = current_drone.setGoal(new_objective);
                end
                new_drones_list{i} = current_drone;
            end
        end
    end

    return

else
    error("Unknown trajectory type: %s", trajectory);
end

end

