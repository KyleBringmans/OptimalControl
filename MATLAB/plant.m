function plant = plant()
%Stuct with att, alt and nav system
plant.attitude = att_lin_system(quat_params);
plant.altitude = alt_lin_system(quat_params);
plant.navigation = nav_lin_system(quat_params);
