clear; close all; clc  
import Drone.*
import Artva.*
import Plotter.*

%% CHOSEN Variables
show_simulation = true;
control_time = 4; %2 for the centralized mode, 1 for the distributed 
global threshold;
%threshold = 0.00001; % --> 1mm PER IL CASO DISTRIBUITO
threshold = 0.0001; %--> 1cm 
global time_step;
time_step = 0.01;
global distributed_estimation_mode;
distributed_estimation_mode = true;
global dronesSetted;
dronesSetted = false;
global trajectory_type;
trajectory_type = "circ"; % Either "circ","patrol","rect"
global see_text;
see_text = false;
global drones_num;
drones_num = 2;

%% Constants
NONE = -1;
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
global last_estimate
last_estimate = [-1;-1];
global control_steps;
control_steps = int16(control_time/time_step);
history_var = zeros(2, control_steps);
check = ones(1, control_steps);
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
    tic %start counting

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
        % est_X = est_X + inv(est_S)*est_H*(est_Y - est_H.'*est_X);
        est_X = est_X + est_S\(est_H*(est_Y - est_H.'*est_X)); % Should be better than previous version
        est_S = est_beta*est_S + est_H * est_H.';
        est_artva.position = [est_X(7), est_X(8), est_X(9)];
        % Save the values to check when the algorithm is not updating the values anymore
        history_var(:,k) = [est_X(7), est_X(8)]';
        [result,check,k,history_var] = stopping_criterium(history_var,check,k);
        if result
            disp("The value of the estimate did not change by " + threshold*100 + " m for more than "+ control_time + "s")
            disp("You have estimated the goal with an error of: " + norm((artva.position'*100) - (est_artva.position'*100)) + " m");
            disp("Simulation stopped at: " + time_instant + " seconds" );
            disp("Goal position:" );
            artva.position
            est_artva.position
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
        
        history_var(:,k) = consensus_mean_array(1,1:2);
        %history_var(:,k) = mean([est_artva_x_array; est_artva_y_array],2);
        if dronesSetted
            [result,check,k,history_var] = stopping_criterium(history_var,check,k); %#ok<UNRCH>
            if result
                disp("The value of the estimate did not change by " + threshold*100 + " m for more than " + control_time + "s")
                disp("You have estimated the goal with a TRUE  error of: " + norm((artva.position(1,1:2)*100) - (mean([est_artva_x_array; est_artva_y_array],2)'*100)) + "m");
                disp("You have estimated the goal with an error wrt CONSENSUs MEAN: " + norm((artva.position(1,1:2)*100) - (consensus_mean_array(:,1)'*100)) + "m");
                disp("Simulation stopped at: " + time_instant + " seconds" );
                disp("Goal position:" );
                artva.position
                consensus_mean_array(:,1)'
                mean([est_artva_x_array; est_artva_y_array],2)'
                break;
            end
        end
    end

    if drones_list{1}.position(2) > 1
        break;
    end

    if(show_simulation)
        if(~distributed_estimation_mode)
            p.draw([drones_x_array; drones_y_array], artva.position, est_artva.position, time_instant);
        else
            p.draw([drones_x_array; drones_y_array], artva.position, [est_artva_x_array; est_artva_y_array],time_instant,consensus_mean_array);
        end
        elapsed_time = toc;
        pause(max(0,time_step-elapsed_time)) %I take out the computational time so that it acually pauses for only one millisecond 
    end
    time_instant = time_instant + time_step;
end

p.close();

%% Functions
function [result,check,k,history_var] = stopping_criterium(history_var,check,k)
result = false;
global threshold;
global control_steps;
global last_estimate;
%sum(check(1,:),'all')
% If for consecutives times the check is always true then stop the simulation
if sum(check(1,:),'all') == 0 && (norm(history_var(:,1) - history_var(:,control_steps)) < threshold)
    result = true;
end

if k > 1
    if norm(history_var(:,k) - history_var(:,k-1)) < threshold
        check(1,k) = 0;
    else 
        check(1,k) = 1;
    end
end

if k == 1
    if norm(history_var(:,1) - last_estimate) < threshold
        check(1,1) = 0;
    else 
        check(1,1) = 1;
    end
end

if k == control_steps
    last_estimate = history_var(:,k);
    if norm(history_var(:,k) - history_var(:,k-1)) < threshold
        check(1,k) = 0;
    else 
        check(1,k) = 1;
    end
    k = 0;
    history_var = zeros(2,control_steps);
end
k = k + 1;
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
    %artva = Artva([rand, rand, 0]);
    artva = Artva([0.9, 0.9, 0]);

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
    %artva = Artva([-1 + 2 * rand,-1 + 2 * rand,0]); %random value between -1 and 1
    artva = Artva([0.9,0.9, 0]);

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

    %artva = Artva([rand, rand, 0]);
    artva = Artva([0.1,0.1, 0]);

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
    dronesSetted = true;

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
    dronesSetted = true;
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

