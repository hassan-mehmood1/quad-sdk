clear all; clc; close all;

addpath('./utils')

%% Physics parameter
parameter.physics.gravitational_constant=9.81; % Gravity

parameter.physics.sim2real_scale_factor=(13.3-11.6620+12.818)/12.818; % Real jamal
% parameter.physics.sim2real_scale_factor=1; % Sim spirit or A1

parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*12.818; % Only body weight of jamal
% parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*6.0; % Only body weight of A1

parameter.physics.mass_body_leg=2.26; % Each leg weight of jamal
% parameter.physics.mass_body_leg=1.935; % Each leg weight of A1

parameter.physics.mass_body=parameter.physics.mass_body_body+...
    4*parameter.physics.mass_body_leg; % Total body weight

parameter.physics.hip_offset=[0.2698; 0.0808; 0]; % Absolute hip offset from body COM of spirit
% parameter.physics.hip_offset=[0.1805; 0.047; 0]; % Absolute hip offset from body COM of A1

% parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
%     diag([0.05; 0.1; 0.1]); % Body inertia of spirit
parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
    [0.05656679 , -8.54731557026013E-06 , 3.411601276539E-05;
    -8.54731557026013E-06 , 0.0364645326398335 , -4.86965244292946E-06;
    3.411601276539E-05, -4.86965244292946E-06 , 0.0737588756462406]; % Body inertia of jamal

parameter.physics.inertia_body=parameter.physics.inertia_body+...
    4*parameter.physics.mass_body_leg*...
    diag([parameter.physics.hip_offset(2)^2+parameter.physics.hip_offset(3)^2;
    parameter.physics.hip_offset(1)^2+parameter.physics.hip_offset(3)^2;
    parameter.physics.hip_offset(1)^2+parameter.physics.hip_offset(2)^2]); % Robot inertia (assume leg mass concentrated at hip)

parameter.name = "jamal"; % Model name
parameter.n = 12; % State dimension
parameter.m = 12; % Input dimension

%% Generate Dynamics Model
dynamicsModel(parameter);
