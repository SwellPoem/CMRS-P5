classdef Drone
    properties
        id
        position
        initial_position
        goal
        state % either "idle", "accelerating", "coasting", "decelerating"

        % trajectory stuff
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

            obj.time_trajectory_instant = obj.time_trajectory_instant + 0.01;

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
    end
end