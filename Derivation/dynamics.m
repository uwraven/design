%% 3D Allocator dynamics
% ARCC Design document
% Matt Vredevoogd 02/27/2020


%% Allocated input dynamics

% actuator inputs and vehicle geometry values
syms F_e G B F_RR F_RP F_RY 'real';
syms l_r1 l_r2 l_e 'real';

Fr_x = F_e * cos(G) * cos(B);
Fr_y = -F_e * cos(B) * sin(G) + F_RY;
Fr_z = F_e * sin(B) + F_RP;
Mr_x = 2 * F_RR * l_r2;
Mr_y = F_RP * l_r1 + F_e * sin(B) * l_e;
Mr_z = F_RY * l_r1 + F_e * cos(B) * sin(G) * l_e;

Ur = [Fr_x Fr_y Fr_z Mr_x Mr_y Mr_z];



%% Direct allocation equations

% global frame control inputs and vehicle geometry values
syms Mr_x Mr_y Mr_z Fr_x Fr_y Fr_z 'real';
syms l_r1 l_r2 l_e 'real';

% This script does not include the EOM derivation

% Body frame forces from RCS
F_RY = (Mr_z - Fr_y * l_e) / (l_r1 - l_e);
F_RP = (Mr_y - Fr_z * l_e) / (l_r1 - l_e);
F_RR = 1 / (2 * l_r2) * Mr_x;

% Gimbal angles and engine thrust
G = atan((F_RY - Fr_y) / Fr_x);
B = atan((Fr_z - F_RP) / Fr_x * cos(G));
F_e = Fr_x / (cos(G) * cos(B));

% Computed allocator outputs, no magnitude / rate limits
U = [F_RY F_RP F_RR G B F_e];

% Ensure that the computed values resolve to the original requests
Fa_x = F_e * cos(G) * cos(B);
Fa_y = -F_e * cos(B) * sin(G) + F_RY;
Fa_z = F_e * sin(B) + F_RP;
Ma_x = 2 * F_RR * l_r2;
Ma_y = F_RP * l_r1 + F_e * sin(B) * l_e;
Ma_z = F_RY * l_r1 + F_e * cos(B) * sin(G) * l_e;
U_a = [Fa_x Fa_y Fa_z Ma_x Ma_y Ma_z];

% Substitude Latex friendly symbols for output
U_L = subs(U, [Mr_x Mr_y Mr_z Fr_x Fr_y Fr_z l_r1 l_r2 l_e], [sym('M_x__r') sym('M_y__r') sym('M_z__r') sym('F_x__r') sym('F_y__r') sym('F_z__r') sym('l_r_1') sym('l_r_2') sym('l_E')]);
Ua_L = subs(U_a, [Mr_x Mr_y Mr_z Fr_x Fr_y Fr_z l_r1 l_r2 l_e], [sym('M_x__r') sym('M_y__r') sym('M_z__r') sym('F_x__r') sym('F_y__r') sym('F_z__r') sym('l_r_1') sym('l_r_2') sym('l_E')]);