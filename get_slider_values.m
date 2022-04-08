function get_slider_values(src,~,gui,u,q0,q,dq,ddq)
    
% Read value and update torque vector
u(str2double(src.String)) = src.Value;

% Update text
gui.config_table.Data =  table2cell(table(q0, q,dq, ddq, u));


end