classdef biped_rl_env < rl.env.MATLABEnvironment
    %Defining a custom environment in MATLAB for a Biped.    
    
    %% Properties 
    properties   
        
        % Physical
        height = 1.8288;
        mass = 75;

        % Calculate model parameters based off height and mass
        model_params = get_model_params(height, mass);
        ctrl_params  = get_ctrl_params();

        % Time step
        dt = 1e-2; 
        
        % Hip height at which to fail the episode [m]
        hip_height_threshold = 0;
        
        % Minimum step length at which to fail the episode
        step_length_threshold = 0
        
        % Reward each time step
        RewardForNotFalling = 1
        
        % Penalty when the biped fails to balance
        PenaltyForFalling = -10 
    end
    
    %% Initialize system state
    properties
        
        % Inital angles
        q = [ 
             0.4709;    % 1 Free Thigh
             -0.7512;   % 2 Between Thighs
             -0.2584 ;  % 3 Free Shank
             -0.3531;   % 4 Back Shank
             1.8002;    % 5 Free Foot
             1.5495;    % 6 Stance Foot
             13/16 * pi % 7 HAT
             ];

         
        [q0000,q000,q00,q0]             = deal(zeros(7,1));
        [dq0000,dq000,dq00,dq0,dq]      = deal(zeros(7,1));
        [ddq0000,ddq000,ddq00,ddq0,ddq] = deal(zeros(7,1));
        [u0000,u000,u00,u0,u]           = deal(zeros(7,1));
        
        % Initialize system state as [q,dq,ddq,u] 
        % including the last 4 time steps (q is at time t, q0 at t-1, q0000
        % at t-4)
        State = [q0000   ,q000   ,q00   ,q0   ,q,...
                 dq0000  ,dq000  ,dq00  ,dq0  ,dq,....
                 ddq0000 ,ddq000 ,ddq00 ,ddq0 ,ddq,...
                 u0000   ,u000   ,u00   ,u0   ,u];
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function env = biped_rl_env()
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([20 7]);
            ObservationInfo.Name = 'Biped States';
            qs   = 'q0000   ,q000   ,q00   ,q0   ,q,';
            dqs  = 'dq0000  ,dq000  ,dq00  ,dq0  ,dq,';
            ddqs = 'ddq0000 ,ddq000 ,ddq00 ,ddq0 ,ddq,';
            us   = 'u0000   ,u000   ,u00   ,u0   ,u';
            ObservationInfo.Description = [qs,dqs,ddqs,us];
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([1 7]);
            ActionInfo.Name = 'Torques';
            ActionInfo.Description = 'u';
            
            % Implement built-in functions for RL env
            env = env@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
            
            % Initialize property values and pre-compute necessary values
            updateActionInfo(env);
        end
        
        % Simulate a step
        function [Observation,Reward,IsDone,LoggedSignals] = step(env,Action)
            % Initialize logging
            LoggedSignals = [];
            
            % Get action
            Force = getForce(env,Action);            
            
            % Unpack state vector
            XDot = env.State(2);
            Theta = env.State(3);
            ThetaDot = env.State(4);
            
            % Cache to avoid recomputation
            CosTheta = cos(Theta);
            SinTheta = sin(Theta);            
            SystemMass = env.CartMass + env.PoleMass;
            temp = (Force + env.PoleMass*env.HalfPoleLength * ThetaDot^2 * SinTheta) / SystemMass;

            % Apply motion equations            
            ThetaDotDot = (env.Gravity * SinTheta - CosTheta* temp) / (env.HalfPoleLength * (4.0/3.0 - env.PoleMass * CosTheta * CosTheta / SystemMass));
            XDotDot  = temp - env.PoleMass*env.HalfPoleLength * ThetaDotDot * CosTheta / SystemMass;
            
            % Euler integration
            Observation = env.State + env.Ts.*[XDot;XDotDot;ThetaDot;ThetaDotDot];

            % Update system states
            env.State = Observation;
            
            % Check terminal condition
            X = Observation(1);
            Theta = Observation(3);
            IsDone = abs(X) > env.DisplacementThreshold || abs(Theta) > env.AngleThreshold;
            env.IsDone = IsDone;
            
            % Get reward
            Reward = getReward(env);
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(env);
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(env)
            % Theta (+- .05 rad)
            T0 = 2 * 0.05 * rand - 0.05;  
            % Thetadot
            Td0 = 0;
            % X 
            X0 = 0;
            % Xdot
            Xd0 = 0;
            
            InitialObservation = [T0;Td0;X0;Xd0];
            env.State = InitialObservation;
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(env);
        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Helper methods to create the environment
        % Discrete force 1 or 2
        function force = getForce(env,action)
            if ~ismember(action,env.ActionInfo.Elements)
                error('Action must be %g for going left and %g for going right.',-env.MaxForce,env.MaxForce);
            end
            force = action;           
        end
        
        % update the action info based on max force
        function updateActionInfo(env)
            env.ActionInfo.Elements = env.MaxForce*[-1 1];
        end
        
        % Reward function
        function Reward = getReward(env)
            if ~env.IsDone
                Reward = env.RewardForNotFalling;
            else
                Reward = env.PenaltyForFalling;
            end          
        end
        
        % (optional) Visualization method
        function plot(env)
            % Initiate the visualization
            
            % Update the visualization
            envUpdatedCallback(env)
        end
        
        % (optional) Properties validation through set methods
        function set.State(env,state)
            validateattributes(state,{'numeric'},{'finite','real','vector','numel',4},'','State');
            env.State = double(state(:));
            notifyEnvUpdated(env);
        end
        function set.HalfPoleLength(env,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','HalfPoleLength');
            env.HalfPoleLength = val;
            notifyEnvUpdated(env);
        end
        function set.Gravity(env,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','Gravity');
            env.Gravity = val;
        end
        function set.CartMass(env,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','CartMass');
            env.CartMass = val;
        end
        function set.PoleMass(env,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','PoleMass');
            env.PoleMass = val;
        end
        function set.MaxForce(env,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','MaxForce');
            env.MaxForce = val;
            updateActionInfo(env);
        end
        function set.Ts(env,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','Ts');
            env.Ts = val;
        end
        function set.AngleThreshold(env,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','AngleThreshold');
            env.AngleThreshold = val;
        end
        function set.DisplacementThreshold(env,val)
            validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','DisplacementThreshold');
            env.DisplacementThreshold = val;
        end
        function set.RewardForNotFalling(env,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','RewardForNotFalling');
            env.RewardForNotFalling = val;
        end
        function set.PenaltyForFalling(env,val)
            validateattributes(val,{'numeric'},{'real','finite','scalar'},'','PenaltyForFalling');
            env.PenaltyForFalling = val;
        end
    end
    
    methods (Access = protected)
        % (optional) update visualization everytime the environment is updated 
        % (notifyEnvUpdated is called)
        function envUpdatedCallback(env)
        end
    end
end
