function controller = init_controller(plant)
%controller contains the K,L and G matrices for att, alt and nav
%structure.

%attitude control matrices
%attitude K
controller.attitude.K = getK_att(plant.attitude);

%attitude L
controller.attitude.L = getL_att(plant.attitude);

%attitude G
controller.attitude.G = getG_att(plant.attitude);


%altitude contorl matrices
%altitude K
[controller.altitude.Kx, controller.altitude.Kz] = getK_alt(plant.altitude);

%altitude L
controller.altitude.L = getL_alt(plant.altitude);

%altitude G
controller.altitude.G = getG_alt(plant.altitude);

%navigation contorl matrices
%altitude K
[controller.navigation.Kx, controller.navigation.Kz] = getK_nav(plant.navigation);

%altitude L
controller.navigation.L = getL_nav(plant.navigation);

%altitude G
controller.navigation.G = getG_nav(plant.navigation);

