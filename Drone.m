classdef Drone
    properties
        id
        position
        initial_position
        goal
        state % Either "idle", "accelerating", "coasting", "decelerating"

        % Decentralized estimation
		est_H
		est_Y
        est_S
        est_beta
        est_X
        est_pos

        % Trajectory stuff
        time_trajectory_instant
        v_max
        a_max
        time_to_full_speed
        time_total
        trajectory_length
    end
    methods
        % Constructor
        function obj = Drone(id, p)
            if nargin == 2
                obj.id = id;
                obj.position = p;
                obj.initial_position = -1;
                obj.goal = -1;
                obj.state = "idle";
                
		        global drones_num;
		        obj.est_H = zeros(10, drones_num);
		        obj.est_Y = zeros(drones_num, 1);
                obj.est_S = eye(10);
                obj.est_beta = 1.0;
                obj.est_X = zeros(10, 1);
                obj.est_pos = zeros(3, 1);

                obj.time_trajectory_instant = 0;
                obj.v_max = 0.05; % 5 m/s
                obj.a_max = 0.01; % 1 m/s
                obj.time_to_full_speed = obj.v_max / obj.a_max;
                obj.trajectory_length = -1;
            end
        end

        function printState(obj)
            fprintf("Drone %d is now %s\n", obj.id, obj.state);
        end

        function obj = setGoal(obj, goal)
            obj.initial_position = obj.position;
            obj.goal = goal;
            obj.time_trajectory_instant = 0;
            obj.trajectory_length = norm(obj.goal - obj.initial_position);
            if(obj.trajectory_length > obj.v_max^2/obj.a_max) % Trajectory profile if there is a coasting phase
                obj.time_total = (obj.trajectory_length * obj.a_max + obj.v_max^2)/(obj.a_max*obj.v_max);
            else % Trajectory profile if there is *not* a coasting phase
                obj.time_total = 2*sqrt(obj.trajectory_length / obj.a_max);
            end
        end

        function b = isAtGoal(obj)
            b = norm(obj.position - obj.goal) <= 0.000001;
        end

        function obj = move(obj)
            global time_step;

            if(obj.time_total <= obj.time_trajectory_instant)
                return
            end

            % Trajectory - time part
            if(obj.trajectory_length>obj.v_max^2/obj.a_max) % Trajectory profile if there is a coasting phase
                if(obj.time_trajectory_instant <= obj.time_to_full_speed)
                    if(obj.state ~= "accelerating")
                        obj.state = "accelerating";
                        obj.printState()
                    end
                    sigma = (obj.a_max * obj.time_trajectory_instant^2)/2;
                elseif(obj.time_trajectory_instant <= obj.time_total - obj.time_to_full_speed)
                    if(obj.state ~= "coasting")
                        obj.state = "coasting";
                        obj.printState()
                    end
                    sigma = obj.v_max * obj.time_trajectory_instant - (obj.v_max^2)/(2 * obj.a_max);
                else
                    if(obj.state ~= "decelerating")
                        obj.state = "decelerating";
                        obj.printState()
                    end
                    sigma = - (obj.a_max*(obj.time_trajectory_instant - obj.time_total)^2)/2 + obj.v_max * obj.time_total - (obj.v_max^2)/(obj.a_max);
                end
            else % Trajectory profile if there is *not* a coasting phase
                if(2 * obj.time_trajectory_instant <= obj.time_total)
                    if(obj.state ~= "accelerating")
                        obj.state = "accelerating";
                        obj.printState()
                    end
                    sigma = (obj.a_max * obj.time_trajectory_instant^2)/2;
                else
                    if(obj.state ~= "decelerating")
                        obj.state = "decelerating";
                        obj.printState()
                    end
                    sigma = - (obj.a_max*(obj.time_trajectory_instant - obj.time_total)^2)/2 + obj.a_max * (obj.time_total/2)^2;
                end
            end

            sigma = sigma / obj.trajectory_length;

            obj.time_trajectory_instant = obj.time_trajectory_instant + time_step;

            % Trajectory - geometric part
            obj.position = obj.goal * sigma + ...
                           obj.initial_position * (1 - sigma);

            if(obj.isAtGoal())
                disp("Reached goal")
                if(obj.state ~= "idle")
                    obj.state = "idle";
                    obj.printState()
                end
                return
            end

        end

        function obj = sync(obj, drones_list)
            
            % Update est_H
            prev_id = obj.id-1;
            next_id = obj.id+1;

            if(prev_id>=1)
                obj.est_H(:, prev_id) = drones_list{prev_id}.est_H(:, prev_id);
                obj.est_Y(prev_id) = drones_list{prev_id}.est_Y(prev_id);
            end
            if(next_id<=size(drones_list, 2))
                obj.est_H(:, next_id) = drones_list{next_id}.est_H(:, next_id);
                obj.est_Y(next_id) = drones_list{next_id}.est_Y(next_id);
            end

            % Update est_S
            obj.est_S = obj.est_beta*obj.est_S + obj.est_H * obj.est_H.';
        end

        function obj = estimate(obj, artva)
            % TODO
            [phi, signal] = artva.getSignal(obj.position);
          
            obj.est_H(:,obj.id) = phi;
            obj.est_Y(obj.id) = signal;
            % est_X = est_X + inv(est_S)*est_H*(est_Y - est_H.'*est_X);
            obj.est_X = obj.est_X + obj.est_S\(obj.est_H*(obj.est_Y - obj.est_H.'*obj.est_X)); % Should be better than previous version
            obj.est_S = obj.est_beta*obj.est_S + obj.est_H * obj.est_H.';
            obj.est_pos(1) = obj.est_X(7);
            obj.est_pos(2) = obj.est_X(8);
            obj.est_pos(3) = obj.est_X(9);
        end
        
    end
end
