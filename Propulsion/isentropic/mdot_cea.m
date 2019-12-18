function [mdot, At] = mdot_cea(F, cf, cstar, P0)
mdot = F / (cstar * cf);
At = cstar * mdot / P0;
end