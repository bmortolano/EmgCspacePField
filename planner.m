classdef planner < handle
% This class combines potential fields from the user_intent class and
% the env_pot_field classes and performs local planning over them.
% Planning is performed by descending a gradient.
    
    methods

        function grad = get_grad(obj, x, net_field) %returns gradient vector. x describes current location on pot field
            x = round(x);
            step = 1;
            if x(1)>1 && x(1)<size(net_field, 1) && x(2)>1 && x(2)<size(net_field, 2) %check if at edges of field
                grad1 = net_field(x(1)+step,x(2))-net_field(x(1),x(2));
                grad2 = net_field(x(1),x(2)+step)-net_field(x(1),x(2));
            else
                grad1 = 0; %improve error handling at some point
                grad2 = 0;
            end

            grad = [grad2, grad1]/step;
        end

        function new_x=descend_grad(obj, x, net_field)%returns next location along path. x describes current location on pot field
            epsilon = 50;
            grad = obj.get_grad(x, net_field);
            if (norm(grad) == 0)
                new_x = x;
            else
                new_x = x - epsilon * grad;
            end
            
            new_x = mod(new_x, 360);
        end
    end
end