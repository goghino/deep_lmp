% Setup the paths
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path to the Matpower library (update to reflect the location at your system)
addpath( ...
    '/Users/Juraj/Documents/DXT/matpower/lib', ...
    '/Users/Juraj/Documents/DXT/matpower/lib/t', ...
    '/Users/Juraj/Documents/DXT/matpower/data', ...
    '/Users/Juraj/Documents/DXT/matpower/most/lib', ...
    '/Users/Juraj/Documents/DXT/matpower/most/lib/t', ...
    '/Users/Juraj/Documents/DXT/matpower/mp-opt-model/lib', ...
    '/Users/Juraj/Documents/DXT/matpower/mp-opt-model/lib/t', ...
    '/Users/Juraj/Documents/DXT/matpower/mips/lib', ...
    '/Users/Juraj/Documents/DXT/matpower/mips/lib/t', ...
    '/Users/Juraj/Documents/DXT/matpower/mptest/lib', ...
    '/Users/Juraj/Documents/DXT/matpower/mptest/lib/t', ...
    '-end' );

% Path to the MOSEK library (update to reflect the location at your system)
addpath('/Users/Juraj/Libraries/mosek/9.2/toolbox/r2015a');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Load the Matpower case and create Matpower options
constants;                    % load some constant definitions
mpc        = case30;          % load the power grid data

assert(all(mpc.branch(:,RATE_A) > 0)) % make sure all the line limits are defined
% to fill the missing values use e.g. mpc.branch(missing,RATE_A) = max(mpc.branch(missing,RATE_A)); 
% to run ED set mpc.branch(:,RATE_A)=0, then all LMP are equal

OPFvoltage = POLAR;           % voltage representation
OPFbalance = POWER;           % type of nodal balance equation
OPFstart   = MPC;             % initial guess
OPFsolver  = MOSEK;           % solver
OPFmodel   = DC;              % grid model

mpopt      = create_options(mpc, OPFsolver, OPFstart, OPFvoltage, OPFbalance, OPFmodel);

rand('seed',1);

%% Perturb the power grid operating point (create a single train/test data point)

% Set the global perturbation (scalar number)
scale_global = 30; % i.e. 30%

% Set the local perturbations (vector of perturbations, one for each load in
% the power grid network)
nLoads = size(mpc.bus, 1);
scale_local = 1 + (2*rand(nLoads,1)-1)/10;  % returns x in 1 + Uniform(-1,1)/10

scale = 1 + (scale_global * scale_local)/100;

% Apply perturbation to each load
mpc.bus(:,PD) = scale.*mpc.bus(:,PD);

%% Solve the OPF using the perturbed data
r = runopf(mpc, mpopt);

%% Extract the pricing information of the energy
%case_info(r);
printpf(r)

% Extract the features
features = mpc.bus(:,PD);

% Extract the QoI
ref_bus = find(mpc.bus(:, BUS_TYPE) == REF);
price   = r.bus(ref_bus, LAM_P);

% Training/test data point (features, response)
sample.in  = features;
sample.out = price;