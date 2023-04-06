classdef planner < handle
% This class combines potential fields from the user_intent class and
% the env_pot_field classes and performs local planning over them.
% Planning is performed by linearly interpolating between the current
% configuration and the smallest-cost configuration within radius r.
%       
%  planner properties:
%   intent_field - Instantiation of user_intent object
%   env_field - Instantiation of env_pot_field class

    properties
        intent_field = user_intent;
        env_field = env_pot_field;
    end
end