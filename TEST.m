
clc; close all; 


env = biped_rl_env;
% validateEnvironment(env)
reset(env);
plot(env)

for i =1:30
    step(env,ones(7,1)');
    pause(1)
end


%% Look for intial velocities, acc and setup reset()

%% Ensure model can learn (implement SAC)

%% Reward functions (with stability)

%% Slip