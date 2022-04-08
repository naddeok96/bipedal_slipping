
function ctrl_params = get_ctrl_params()


ctrl_params.c      = [-0.8743046052, 0, -0.4700546052,        0, -0.23925, 0, 0];
ctrl_params.c_slip = [            3, 0,          -3.3,        0,        0, 0, 0];  
ctrl_params.c_di   = [-0.8743046052, 0, -0.4700546052,           -0.23925,    0];       

% Used to map q to the Bezier polynomial
ctrl_params.H0 = [0, 1, 0, 0, 0, 0, 0;
                  0, 0, 1, 0, 0, 0, 0;
                  0, 0, 0, 1, 0, 0, 0;
                  0, 0, 0, 0, 1, 0, 0;
                  0, 0, 0, 0, 0, 1, 0;
                  0, 0, 0, 0, 0, 0, 1];
                  
% di coordinates (q_di = [q1 q2 q3 q5 q7])
ctrl_params.H0_di = [0, 1, 0, 0, 0;
                     0, 0, 1, 0, 0;
                     0, 0, 0, 1, 0;
                     0, 0, 0, 0, 1];  
                     
% Base to di transformation matrix (q_di = Omega * q)
ctrl_params.Omega=[1 0 0 0 0 0 0; 
                   0 1 0 0 0 0 0;
                   0 0 1 0 0 0 0;
                   0 0 0 0 1 0 0;
                   0 0 0 0 0 0 1;
                   0 0 0 1 0 0 0;
                   0 0 0 0 0 1 0]; 

% di to base transformation matrix (q = inv(Omega) * q_di)
ctrl_params.inv_Omega = inv(ctrl_params.Omega);

% Maps u to joint coordinates                 
% ctrl_params.B = [0, 0, 0, 0, 0, 0; % Orginal 7link used this 7x6, unknown
% why, if this comment is still here please debate with advisor
%                  1, 0, 0, 0, 0, 0;
%                  0, 1, 0, 0, 0, 0;
%                  0, 0, 1, 0, 0, 0;
%                  0, 0, 0, 1, 0, 0;
%                  0, 0, 0, 0, 1, 0;
%                  0, 0, 0, 0, 0, 1];
ctrl_params.B = eye(7);
                 