function [dsys_a, dsys_b, dsys_c] = Discretize(csys, sample_period)

dsys = c2d(csys, sample_period, 'zoh');
dsys_a = dsys.a;
dsys_b = dsys.b;
dsys_c = dsys.c;