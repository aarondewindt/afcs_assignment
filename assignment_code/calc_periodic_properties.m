function [ period, t_12 ] = calc_periodic_properties( w_n, zeta )
%CALC_PERIODIC_PROPERTIES Calculates the period and time to damp to half amplitude
    % These equations where obtained from the AE3202 Flight Dynamics
    % lecture notes.
    period = 2*pi./w_n;
    t_12 = period .* 0.110 .* sqrt(1-zeta.^2)./zeta;
end

