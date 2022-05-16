% used in tests of,
% - non-contiguous bus ids
% - tranformer orentation swapping
% - dual values
% - clipping cost functions using ncost
% - linear objective function

function mpc = case5
mpc.version = '2';
mpc.baseMVA = 100.0;

%% bus data
%	bus_i	type	Pd	Qd	Gs	Bs	area	Vm	Va	baseKV	zone	Vmax	Vmin
mpc.bus = [
	1	 2	 0.0	 0.0	 0.0	 0.0	 1	    1.00000	    2.80377	 230.0	 1	    1.10000	    0.90000;
	2	 1	 300.0	 98.61	 0.0	 0.0	 1	    1.08407	   -0.73465	 230.0	 1	    1.10000	    0.90000;
	3	 2	 300.0	 98.61	 0.0	 0.0	 1	    1.00000	   -0.55972	 230.0	 1	    1.10000	    0.90000;
	4	 3	 400.0	 131.47	 0.0	 0.0	 1	    1.00000	    0.00000	 230.0	 1	    1.10000	    0.90000;
	10	 2	 0.0	 0.0	 0.0	 0.0	 1	    1.00000	    3.59033	 230.0	 1	    1.10000	    0.90000;
	6	 1	 0.0	 0.0	 0.0	 0.0	 1	    1.00000	    3.59033	 230.0	 1	    1.10000	    0.90000;
];

%% generator data
%	bus	Pg	Qg	Qmax	Qmin	Vg	mBase	status	Pmax	Pmin
mpc.gen = [
	1	 40.0	 30.0	 30.0	 -30.0	 1.07762	 100.0	 1	 40.0	 0.0;
	1	 170.0	 127.5	 127.5	 -127.5	 1.07762	 100.0	 1	 170.0	 0.0;
	3	 324.498	 390.0	 390.0	 -390.0	 1.1	 100.0	 1	 520.0	 0.0;
	4	 0.0	 -10.802	 150.0	 -150.0	 1.06414	 100.0	 1	 200.0	 0.0;
	10	 470.694	 -165.039	 450.0	 -450.0	 1.06907	 100.0	 1	 600.0	 0.0;
];

%% generator cost data
%	2	startup	shutdown	n	c(n-1)	...	c0
mpc.gencost = [
	2	 0.0	 0.0	 3	   0.000000	  14.000000	   0.000000	   2.000000;
	2	 0.0	 0.0	 3	   0.000000	  15.000000	   0.000000	   2.000000;
	2	 0.0	 0.0	 3	   0.000000	  30.000000	   0.000000	   2.000000;
	2	 0.0	 0.0	 3	   0.000000	  40.000000	   0.000000	   2.000000;
	2	 0.0	 0.0	 3	   0.000000	  10.000000	   0.000000	   2.000000;
];

%% branch data
%	fbus	tbus	r	x	b	rateA	rateB	rateC	ratio	angle	status	angmin	angmax
mpc.branch = [
1	 2	 0.00281	 0.0281	 0.00712	 400.0	 400.0	 400.0	 0.0	  0.0	 1	 -30.0	 30.0;
1	 4	 0.00304	 0.0304	 0.00658	 426	 426	 426	 0.0	  0.0	 1	 -30.0	 30.0;
1	 10	 0.00064	 0.0064	 0.03126	 426	 426	 426	 0.0	  0.0	 1	 -30.0	 30.0;
2	 3	 0.00108	 0.0108	 0.01852	 426	 426	 426	 0.0	  0.0	 1	 -30.0	 30.0;
3	 4	 0.00297	 0.0297	 0.00674	 426	 426	 426	0.0	  0.0	 1	 -30.0	 30.0;
4	 6	 0.00200	 0.0200	 0.00674	 426	 426	 426	 0.0	 0.0	 1	 -30.0	 30.0;
4	 10	 0.00297	 0.0297	 0.00674	 240.0	 240.0	 240.0	 0.0	  0.0	 1	 -30.0	 30.0;
];

%% pst data
%column_names%	fbus	tbus	br_r	br_x	br_b	rateA	rateB	rateC	angle	status	angmin	angmax
mpc.pst = [
	6	 3	 0.00097	 0.0097	 0.0	 426.0	 426.0	 426.0	  0.0	 1	 -30	 30;
];


%% dc grid topology
%colunm_names% dcpoles
mpc.dcpol=2;
% numbers of poles (1=monopolar grid, 2=bipolar grid)
%% bus data
%column_names%   busdc_i grid    Pdc     Vdc     basekVdc    Vdcmax  Vdcmin  Cdc
mpc.busdc = [
    1              1       0       1       345         1.1     0.9     0;
    2              1       0       1       345         1.1     0.9     0;
	3              1       0       1       345         1.1     0.9     0;
];

%% converters
%column_names%   busdc_i busac_i type_dc type_ac P_g   Q_g islcc  Vtar    rtf xtf  transformer tm   bf filter    rc      xc  reactor   basekVac    Vmmax   Vmmin   Imax    status   LossA LossB  LossCrec LossCinv  droop      Pdcset    Vdcset  dVdcset Pacmax Pacmin Qacmax Qacmin
mpc.convdc = [
    1       2   1       1       -60    -40    0 1     0.01  0.01 1 1 0.01 1 0.01   0.01 1  345         1.1     0.9     1.1     1       1.103 0.887  2.885    2.885      0.0050    -58.6274   1.0079   0 100 -100 50 -50;
    2       3   2       1       0       0     0 1     0.01  0.01 1 1 0.01 1 0.01   0.01 1  345         1.1     0.9     1.1     1       1.103 0.887  2.885    2.885      0.0070     21.9013   1.0000   0 100 -100 50 -50;
    3       5   1       1       35       5    0 1     0.01  0.01 1 1 0.01 1 0.01   0.01 1  345         1.1     0.9     1.1     1       1.103 0.887  2.885    2.885      0.0050     36.1856   0.9978   0 100 -100 50 -50;
%		3       5   1       1       35       5    0 1     0.01  0.01 1 1 0.01 1 0.01   0.01 1  345         1.1     0.9     1.1     0       1.103 0.887  2.885    2.885      0.0050     36.1856   0.9978   0 100 -100 50 -50;
];

%% branches
%column_names%   fbusdc  tbusdc  r      l        c   rateA   rateB   rateC   status
mpc.branchdc = [
    1       2       0.052   0   0    100     100     100     1;
    2       3       0.052   0   0    100     100     100     1;
    1       3       0.073   0   0    100     100     100     1;
%		1       3       0.073   0   0    100     100     100     0;
 ];

