classdef planner < handle
% This class combines potential fields from the user_intent class and
% the env_pot_field classes and performs local planning over them.
% Planning is performed by descending a gradient.
    
    methods

        function grad = get_grad(obj, x, net_field) %returns gradient vector. x describes current location on pot field
            x = round(x);

            if x(1)>359
                x(1)=359;
            end

            if x(2)>359
                x(2)=359;
            end

            if x(1)<1
                x(1)=1;
            end

            if x(2)<1
                x(2)=1;
            end

            grad1 = net_field(x(2)+1,x(1))-net_field(x(2),x(1));
            grad2 = net_field(x(2),x(1)+1)-net_field(x(2),x(1));

            grad = [grad2, grad1];
        end

        function new_x=descend_grad(obj, x, net_field)%returns next location along path. x describes current location on pot field
            epsilon = 40;
            grad = obj.get_grad(x, net_field);
            if (norm(grad) == 0)
                new_x = x;
            else
                new_x = x - epsilon * grad;
            end
            
            new_x = mod(new_x, 360);
            
            if any(isnan(new_x)) || any(isinf(new_x))
                new_x = x;
            end
        end
    end
end