classdef planner < handle
% This class combines potential fields from the user_intent class and
% the env_pot_field classes and performs local planning over them.
% Planning is performed by descending a gradient.
% 
%       
%  planner properties:
%   intent_field - Instantiation of user_intent object
%   env_field - Instantiation of env_pot_field class

    properties
<<<<<<< HEAD
        intent_field = user_intent_file_wrapper;
        env_field = env_pot_field;
=======
        intent_field = user_intent;
        env_field = env_pot_field([100 100]);
        net_field = intent_field.field+env_field.field; %cummulative potential field
    end
    
    methods

        function grad = get_grad(obj,x) %returns gradient vector. x describes current location on pot field
            if x(1)>1 && x(1)<=obj.net_field.size(1) && x(2)>1 && x(2)<=obj.net_field.size(2) %check if at edges of field
                grad1=obj.net_field(x(1)+1,x(2))-obj.net_field(x(1),x(2));
                grad2=obj.net_field(x(1),x(2)+1)-obj.net_field(x(1),x(2));
            else
                grad1=0; %improve error handling at some point
                grad2=0;
            end

            grad=[grad1,grad2];
        end

        function new_x=descend_grad(x)%returns next location along path. x describes current location on pot field
            grad=get_grad(x);
            new_x=round(x-grad/norm(grad));
        end
>>>>>>> 963ae824b9686ff82268be4095fd6b9d538975c4
    end
end