function F = Function_obj(tf,init_p,final_p,passage)

syms a1_0 a1_1 a1_2 a1_3 a1_4 a1_5 
syms a2_0 a2_1 a2_2 a2_3 a2_4 a2_5 a2_6 a2_7 a2_8 a2_9
syms a3_0 a3_1 a3_2 a3_3 a3_4 a3_5 a3_6 a3_7 a3_8 a3_9
syms a4_0 a4_1 a4_2 a4_3 a4_4 a4_5 
syms t tend

t0 = 0;

%Parameters (UAVs parameters are taken from reference [18]
g = 9.81;
l = 0.2;
C = 0.1;
J1 = 1.25;
J2 = 1.25;
J3 = 2.5;
T_max =10;       
m = 2;

%Boundary conditions
dF1_tend = 0.5;
dF2_tend = 0.5;
dF3_tend = 0.5;
dF4_tend = 0;

ddF1_tend = 0.2;
ddF2_tend =0.2;
ddF3_tend = 0.2;
ddF4_tend = 0;

d3F1_tend = 0;
d3F2_tend = 0;
d3F3_tend = 0;
d3F4_tend = 0;
d4F1_tend = 0;
d4F2_tend = 0;
d4F3_tend = 0;
d4F4_tend = 0;

switch passage
    case 0 
        init_cond = [init_p(1,1) init_p(1,2) init_p(1,3) init_p(1,4) 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
        f_cond = [final_p(1,1) final_p(1,2) final_p(1,3) final_p(1,4) dF1_tend dF2_tend dF3_tend dF4_tend ddF1_tend ddF2_tend ddF3_tend ddF4_tend d3F1_tend d3F2_tend d3F3_tend d3F4_tend d4F1_tend d4F2_tend d4F3_tend d4F4_tend];
    case 1 
        init_cond = [init_p(1,1) init_p(1,2) init_p(1,3) init_p(1,4) dF1_tend dF2_tend dF3_tend dF4_tend ddF1_tend ddF2_tend ddF3_tend ddF4_tend d3F1_tend d3F2_tend d3F3_tend d3F4_tend d4F1_tend d4F2_tend d4F3_tend d4F4_tend];
        f_cond = [final_p(1,1) final_p(1,2) final_p(1,3) final_p(1,4) dF1_tend dF2_tend dF3_tend dF4_tend ddF1_tend ddF2_tend ddF3_tend ddF4_tend d3F1_tend d3F2_tend d3F3_tend d3F4_tend d4F1_tend d4F2_tend d4F3_tend d4F4_tend];
    case 2
        init_cond = [init_p(1,1) init_p(1,2) init_p(1,3) init_p(1,4) dF1_tend dF2_tend dF3_tend dF4_tend ddF1_tend ddF2_tend ddF3_tend ddF4_tend d3F1_tend d3F2_tend d3F3_tend d3F4_tend d4F1_tend d4F2_tend d4F3_tend d4F4_tend];
        f_cond = [final_p(1,1) final_p(1,2) final_p(1,3) final_p(1,4) 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    case 3
        init_cond = [init_p(1,1) init_p(1,2) init_p(1,3) init_p(1,4) 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
        f_cond = [final_p(1,1) final_p(1,2) final_p(1,3) final_p(1,4) 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
end


F1_t0 =a1_0;
F2_t0 =a2_0;
F3_t0 =a3_0;
F4_t0 =a4_0;
dF1_t0 =a1_1;
dF2_t0 =a2_1;
dF3_t0 =a3_1;
dF4_t0 = a4_1;
ddF1_t0 = 2*a1_2;
ddF2_t0 = 2*a2_2;
ddF3_t0 = 2*a3_2;
ddF4_t0 = 2*a4_2;
d3F2_t0 = 6*a2_3;
d3F3_t0 = 6*a3_3;
d4F2_t0 = 24*a2_4;
d4F3_t0 = 24*a3_4;

F1_tend = a1_5*tend^5 + a1_4*tend^4 + a1_3*tend^3 + a1_2*tend^2 + a1_1*tend + a1_0;
F2_tend = a2_9*tend^9 + a2_8*tend^8 + a2_7*tend^7 + a2_6*tend^6 + a2_5*tend^5 + a2_4*tend^4 + a2_3*tend^3 + a2_2*tend^2 + a2_1*tend + a2_0;
F3_tend = a3_9*tend^9 + a3_8*tend^8 + a3_7*tend^7 + a3_6*tend^6 + a3_5*tend^5 + a3_4*tend^4 + a3_3*tend^3 + a3_2*tend^2 + a3_1*tend + a3_0;
F4_tend = a4_5*tend^5 + a4_4*tend^4 + a4_3*tend^3 + a4_2*tend^2 + a4_1*tend + a4_0;
dF1_tend = 5*a1_5*tend^4 + 4*a1_4*tend^3 + 3*a1_3*tend^2 + 2*a1_2*tend + a1_1;
dF2_tend = 9*a2_9*tend^8 + 8*a2_8*tend^7 + 7*a2_7*tend^6 + 6*a2_6*tend^5 + 5*a2_5*tend^4 + 4*a2_4*tend^3 + 3*a2_3*tend^2 + 2*a2_2*tend + a2_1;
dF3_tend = 9*a3_9*tend^8 + 8*a3_8*tend^7 + 7*a3_7*tend^6 + 6*a3_6*tend^5 + 5*a3_5*tend^4 + 4*a3_4*tend^3 + 3*a3_3*tend^2 + 2*a3_2*tend + a3_1;
dF4_tend = 5*a4_5*tend^4 + 4*a4_4*tend^3 + 3*a4_3*tend^2 + 2*a4_2*tend + a4_1;
ddF1_tend = 20*a1_5*tend^3 + 12*a1_4*tend^2 + 6*a1_3*tend + 2*a1_2;
ddF2_tend = 72*a2_9*tend^7 + 56*a2_8*tend^6 + 42*a2_7*tend^5 + 30*a2_6*tend^4 + 20*a2_5*tend^3 + 12*a2_4*tend^2 + 6*a2_3*tend + 2*a2_2;
ddF3_tend = 72*a3_9*tend^7 + 56*a3_8*tend^6 + 42*a3_7*tend^5 + 30*a3_6*tend^4 + 20*a3_5*tend^3 + 12*a3_4*tend^2 + 6*a3_3*tend + 2*a3_2;
ddF4_tend = 20*a4_5*tend^3 + 12*a4_4*tend^2 + 6*a4_3*tend + 2*a4_2;
d3F1_tend = 60*a1_5*tend^2 + 24*a1_4*tend + 6*a1_3;
d3F2_tend = 504*a2_9*tend^6 + 336*a2_8*tend^5 + 210*a2_7*tend^4 + 120*a2_6*tend^3 + 60*a2_5*tend^2 + 24*a2_4*tend + 6*a2_3;
d3F3_tend = 504*a3_9*tend^6 + 336*a3_8*tend^5 + 210*a3_7*tend^4 + 120*a3_6*tend^3 + 60*a3_5*tend^2 + 24*a3_4*tend + 6*a3_3;
d3F4_tend = 60*a4_5*tend^2 + 24*a4_4*tend + 6*a4_3;
d4F1_tend = 24*a1_4 + 120*a1_5*tend;
d4F2_tend = 3024*a2_9*tend^5 + 1680*a2_8*tend^4 + 840*a2_7*tend^3 + 360*a2_6*tend^2 + 120*a2_5*tend + 24*a2_4;
d4F3_tend = 3024*a3_9*tend^5 + 1680*a3_8*tend^4 + 840*a3_7*tend^3 + 360*a3_6*tend^2 + 120*a3_5*tend + 24*a3_4;
d4F4_tend = 24*a4_4 + 120*a4_5*tend;

%Initial conditions on position
i1 = F1_t0 == init_cond(1);
i2 = F2_t0 == init_cond(2);
i3 = F3_t0 == init_cond(3);
i4 = F4_t0 == init_cond(4);

%Initial conditions on velocity
i5 = dF1_t0 == init_cond(5);
i6 = dF2_t0 == init_cond(6);
i7 = dF3_t0 == init_cond(7);
i8 = dF4_t0 == init_cond(8);

%Initial conditions on accelerations
i9 = ddF1_t0 == init_cond(9);
i10 = ddF2_t0 == init_cond(10);
i11 = ddF3_t0 == init_cond(11);
i12 = ddF4_t0 == init_cond(12);

%Initial conditions on jerk
i13 = d3F2_t0 == init_cond(13);
i14 = d3F3_t0 == init_cond(14);

%Initial conditions on jounce
i15 = d4F2_t0 == init_cond(15);
i16 = d4F3_t0 == init_cond(16);

%Final conditions on position
f1 = F1_tend == f_cond(1);
f2 = F2_tend == f_cond(2);
f3 = F3_tend == f_cond(3);
f4 = F4_tend == f_cond(4);

%Final conditions on velocity
f5 = dF1_tend == f_cond(5);
f6 = dF2_tend == f_cond(6);
f7 = dF3_tend == f_cond(7);
f8 = dF4_tend == f_cond(8);

%Final conditions on accelerations
f9 = ddF1_tend == f_cond(9);
f10 = ddF2_tend == f_cond(10);
f11 = ddF3_tend == f_cond(11);
f12 = ddF4_tend == f_cond(12);

%Final conditions on jerk
f13 = d3F2_tend == f_cond(13);
f14 = d3F3_tend == f_cond(14);

%Final conditions on jounce
f15 = d4F2_tend == f_cond(15);
f16 = d4F3_tend == f_cond(16);

eqs = [i1 i2 i3 i4 i5 i6 i7 i8 i9 i10 i11 i12 i13 i14 i15 i16 f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 f12 f13 f14 f15 f16];
vars = [a1_0 a1_1 a1_2 a1_3 a1_4 a1_5 a2_0 a2_1 a2_2 a2_3 a2_4 a2_5 a2_6 a2_7 a2_8 a2_9 a3_0 a3_1 a3_2 a3_3 a3_4 a3_5 a3_6 a3_7 a3_8 a3_9 a4_0 a4_1 a4_2 a4_3 a4_4 a4_5];
parameters = solve(eqs,vars);
params = [parameters.a1_0 parameters.a1_1 parameters.a1_2 parameters.a1_3 parameters.a1_4 parameters.a1_5 parameters.a2_0 parameters.a2_1 parameters.a2_2 parameters.a2_3 parameters.a2_4 parameters.a2_5 parameters.a2_6 parameters.a2_7 parameters.a2_8 parameters.a2_9 parameters.a3_0 parameters.a3_1 parameters.a3_2 parameters.a3_3 parameters.a3_4 parameters.a3_5 parameters.a3_6 parameters.a3_7 parameters.a3_8 parameters.a3_9 parameters.a4_0 parameters.a4_1 parameters.a4_2 parameters.a4_3 parameters.a4_4 parameters.a4_5]';

a1_0 = params(1);
a1_1 = params(2);
a1_2 = params(3);
a1_3 = params(4);
a1_4 = params(5);
a1_5 = params(6);

a2_0 = params(7);
a2_1 = params(8);
a2_2 = params(9);
a2_3 = params(10);
a2_4 = params(11);
a2_5 = params(12);
a2_6 = params(13);
a2_7 = params(14);
a2_8 = params(15);
a2_9 = params(16);

a3_0 = params(17);
a3_1 = params(18);
a3_2 = params(19);
a3_3 = params(20);
a3_4 = params(21);
a3_5 = params(22);
a3_6 = params(23);
a3_7 = params(24);
a3_8 = params(25);
a3_9 = params(26);

a4_0 = params(27);
a4_1 = params(28);
a4_2 = params(29);
a4_3 = params(30);
a4_4 = params(31);
a4_5 = params(32);


ddF1 = 20*a1_5*t^3 + 12*a1_4*t^2 + 6*a1_3*t + 2*a1_2;
ddF4 = 20*a4_5*t^3 + 12*a4_4*t^2 + 6*a4_3*t + 2*a4_2;
d4F2 = 3024*a2_9*t^5 + 1680*a2_8*t^4 + 840*a2_7*t^3 + 360*a2_6*t^2 + 120*a2_5*t + 24*a2_4;
d4F3 = 3024*a3_9*t^5 + 1680*a3_8*t^4 + 840*a3_7*t^3 + 360*a3_6*t^2 + 120*a3_5*t + 24*a3_4;

%Nominal Control Inputs
nu1 = ddF1 + g;
nu2 = d4F2/g;
nu3 = -d4F3/g;
nu4 = ddF4;

%Maximal Control Inputs
u1_max = 4*T_max/m;
u2_max = 2*l*T_max/J1;
u3_max = 2*l*T_max/J2;
u4_max = 2*C*T_max/J3;

%Normalized Control Inputs
u1 = nu1/u1_max;
u2 = nu2/u2_max;
u3 = nu3/u3_max;
u4 = nu4/u4_max;

%Energy computation
dE = 4*(u1^2) + (u2^2) + (u3^2) + (u4^2);
J = int(dE, t,t0, tend);
J_val = subs(J,tend,tf);


%Objective function of minimal energy
F = double(J_val);
disp(tf)
clear a1_0 a1_1 a1_2 a1_3 a1_4 a1_5 a2_0 a2_1 a2_2 a2_3 a2_4 a2_5 a2_6 a2_7 a2_8 a2_9 a3_0 a3_1 a3_2 a3_3 a3_4 a3_5 a3_6 a3_7 a3_8 a3_9a4_0 a4_1 a4_2 a4_3 a4_4 a4_5 
clear t tend
end 