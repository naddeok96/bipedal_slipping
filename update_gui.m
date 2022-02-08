function update_gui(gui, q0, q, dq, ddq, u)
    ue = [u(1:5); 0; u(6)];
    gui.config_table.Data =  table2cell(table(q0, q,dq, ddq, ue));
end