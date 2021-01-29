function    Build_trajectory(path,vector_time,p,opt_CT,opt_ST,obstacle)
syms t 

%Parameters (UAVs parameters are taken from reference [18]
g = 9.81;
l = 0.2;
C_1 = 0.1;
J1 = 1.25;
J2 = 1.25;
J3 = 2.5;
T_max =10;
m = 2;

k1=1;
k2=1;
k3=1;

%Set boundary conditions
vel=0.5;
acc=0.2;

function_array = [];
F4_func_array = [];
sss = size(path);
sss = sss(1);

%Compute Bèzier polynomials parameters
for z=1:(size(path)-1)
    syms a1_0 a1_1 a1_2 a1_3 a1_4 a1_5 
    syms a2_0 a2_1 a2_2 a2_3 a2_4 a2_5 a2_6 a2_7 a2_8 a2_9
    syms a3_0 a3_1 a3_2 a3_3 a3_4 a3_5 a3_6 a3_7 a3_8 a3_9
    syms a4_0 a4_1 a4_2 a4_3 a4_4 a4_5 
    syms t tend
    
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

    if z == 1
        init_cond = [path(z,1)   path(z,2)   path(z,3)   0 0   0   0   0 0   0   0   0 0 0 0 0 0 0 0 0];
    else
        init_cond = [path(z,1)   path(z,2)   path(z,3)   0 vel vel vel 0 acc acc acc 0 0 0 0 0 0 0 0 0];
    end
    if z == (sss-1)
        f_cond    = [path(z+1,1) path(z+1,2) path(z+1,3) 0 0   0   0   0 0   0   0   0 0 0 0 0 0 0 0 0];
    else
        f_cond    = [path(z+1,1) path(z+1,2) path(z+1,3) 0 vel vel vel 0 acc acc acc 0 0 0 0 0 0 0 0 0];
    end 
    
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


    F1 = a1_5*t^5 + a1_4*t^4 + a1_3*t^3 + a1_2*t^2 + a1_1*t + a1_0;
    F2 = a2_9*t^9 + a2_8*t^8 + a2_7*t^7 + a2_6*t^6 + a2_5*t^5 + a2_4*t^4 + a2_3*t^3 + a2_2*t^2 + a2_1*t + a2_0;
    F3 = a3_9*t^9 + a3_8*t^8 + a3_7*t^7 + a3_6*t^6 + a3_5*t^5 + a3_4*t^4 + a3_3*t^3 + a3_2*t^2 + a3_1*t + a3_0;
    F4 = a4_5*t^5 + a4_4*t^4 + a4_3*t^3 + a4_2*t^2 + a4_1*t + a4_0;
    
    F1_tend = subs(F1,tend,vector_time(z));
    F2_tend = subs(F2,tend,vector_time(z));
    F3_tend = subs(F3,tend,vector_time(z));
    F4_tend = subs(F4,tend,vector_time(z));
    
    function_array = [function_array F1_tend F2_tend F3_tend ];
    F4_func_array = [F4_func_array F4_tend];
end

syms t

%Plot parameters
hold on
grid on
rotate3d
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

%Plot waypoints
for j=1:size(path)
    plot3(path(j,1),path(j,2),path(j,3),'x')
end


%Plot reference trajectory
sum = [0 0 0];
for i=1:size(vector_time')
    k=(i-1)*3;
    f1 = @(t) function_array(k+1);
    f2 = @(t) function_array(k+2);
    f3 = @(t) function_array(k+3);
    fplot3(f1(t), f2(t), f3(t), [0,vector_time(i)],'b-')
    
    %Util to compute new trajectory
    f1_old = double(subs(function_array(k+1),t,vector_time(i)));
    f2_old = double(subs(function_array(k+2),t,vector_time(i)));
    f3_old = double(subs(function_array(k+3),t,vector_time(i)));
    sum(1) = sum(1) + f1_old;
    sum(2) = sum(2) + f2_old;
    sum(3) = sum(3) + f3_old;
end
view(3)

%Import and plot obstacles
obs_temp=load(obstacle);
num_obs = size(obs_temp,1)/4;
for i=1:num_obs
    obs(:,:,i)=obs_temp(i*4-3:i*4,:);
    
end
hold on

syms q

%Plot quadrotor: cycle for all path's sections
for i=1:size(vector_time')
    k=(i-1)*3;
    f1 = @(t) function_array(k+1);
    f2 = @(t) function_array(k+2);
    f3 = @(t) function_array(k+3);
    df1 = diff(f1,t);
    df2 = diff(f2,t);
    df3 = diff(f3,t);
    ddf2 = diff(df2,t);

    %Cicle for each path section
    for x=0:0.2:vector_time(i)
       
        center_quad = [ double(subs(function_array(k+1),t,x))  double(subs(function_array(k+2),t,x))   double(subs(function_array(k+3),t,x))];
        df1_val = double(subs(df1,t,x));
        df2_val = double(subs(df2,t,x));
        df3_val = double(subs(df3,t,x));
        ddf2_val = double(subs(ddf2,t,x));
        
        %Compute theta and phi angles
        theta = ((ddf2_val + ((k1/m)*df2_val))/(df1_val + ((k3/m)*df1_val + g)));
        phi = -((df3_val + ((k2/m)*df3_val))/(df1_val + ((k3/m) + df1_val + g)));
        
        %Compute points on the axis
        A = [ center_quad(1)               center_quad(2)+l*cos(phi)  center_quad(3)+l*sin(phi)  ];
        B = [ center_quad(1)+l*cos(theta)  center_quad(2)             center_quad(3)+l*sin(theta)];
        C = [ center_quad(1)               center_quad(2)-l*cos(phi)  center_quad(3)-l*sin(phi)  ];
        D = [ center_quad(1)-l*cos(theta)  center_quad(2)             center_quad(3)-l*sin(theta)];
        
        %Plot drone's axis
        h1 = plot3([A(1),C(1)],[A(2),C(2)],[A(3),C(3)],'b');
        h2 = plot3([B(1),D(1)],[B(2),D(2)],[B(3),D(3)],'b');
        
        %Plot drone's propellers  
        r=0.1;
        teta=-pi:0.01:pi;
        %1
        x3=r*cos(teta) + A(1);
        y3=r*sin(teta) + A(2);
        z3 = repelem(A(3),numel(x3));
        h3 = plot3(x3,y3,z3,'r');
        rotate(h3,[0,1,0],phi);
        rotate(h3,[1,0,0],theta);
        %2
        x4=r*cos(teta) + B(1);
        y4=r*sin(teta) + B(2);
        z4 = repelem(B(3),numel(x4));
        h4 = plot3(x4,y4,z4,'r');
        rotate(h4,[0,1,0],phi);
        rotate(h4,[1,0,0],theta);
        %3
        x5=r*cos(teta) + C(1);
        y5=r*sin(teta) + C(2);
        z5 = repelem(C(3),numel(x5));
        h5 = plot3(x5,y5,z5,'r');
        rotate(h5,[0,1,0],phi);
        rotate(h5,[1,0,0],theta);
        %4
        x6=r*cos(teta) + D(1);
        y6=r*sin(teta) + D(2);
        z6 = repelem(D(3),numel(x6));
        h6 = plot3(x6,y6,z6,'r');
        rotate(h6,[0,1,0],phi);
        rotate(h6,[1,0,0],theta);
        
        %Plot quadrotor center
        pause(0.0001)
        %set(h0,'Visible','off')
        set(h1,'Visible','off')
        set(h2,'Visible','off')
        set(h3,'Visible','off')
        set(h4,'Visible','off')
        set(h5,'Visible','off')
        set(h6,'Visible','off')
    end

end

hold off
hold on
vec_time=vector_time;
vettore_thrust=[];
vettore_velocities = [];
vettore_acceleration = [];
for i=1:size(vec_time')
    k=(i-1)*3;
    f1 = @(t) function_array(k+1);
    f2 = @(t) function_array(k+2);
    f3 = @(t) function_array(k+3);
    f4 = @(t) F4_func_array(i);
    
    %Utils derivative
    df1 = diff(f1,t);
    ddf1 = diff(df1,t); 
    df2 = diff(f2,t);
    ddf2 = diff(df2,t);
    d3f2 = diff(ddf2,t);
    d4f2 = diff(d3f2,t); 
    df3 = diff(f3,t);
    ddf3 = diff(df3,t);
    d3f3 = diff(ddf3,t);
    d4f3 = diff(d3f3,t);
    df4 = diff(f4,t);
    ddf4 = diff(df4,t);
    
    %Nominal Control Inputs
    nu1 = ddf1 + g;
    nu2 = d4f2/g;
    nu3 = -d4f3/g;
    nu4 = ddf4;
    
    %Maximal Control Inputs
    u1_max = 4*T_max/m;
    u2_max = 2*l*T_max/J1;
    u3_max = 2*l*T_max/J2;
    u4_max = 2*C_1*T_max/J3;
    
    %Normalized Control Inputs
    u1 = nu1/u1_max;
    u2 = nu2/u2_max;
    u3 = nu3/u3_max;
    u4 = nu4/u4_max;
    
    %Thrusts
    T1 = u1 - (1/2)*u2 - (1/2)*u3 + (1/2)*u4;
    T2 = u1 - (1/2)*u2 + (1/2)*u3 - (1/2)*u4;
    T3 = u1 + (1/2)*u2 + (1/2)*u3 + (1/2)*u4;
    T4 = u1 + (1/2)*u2 - (1/2)*u3 - (1/2)*u4;
    vettore_thrust = [ vettore_thrust T1 T2 T3 T4];
    vettore_velocities = [ vettore_velocities df1 df2 df3 df4 ];
    vettore_acceleration = [ vettore_acceleration ddf1 ddf2 ddf3 ddf4 ];
end

disp(vector_time);
disp(vettore_thrust);

plot_thrust(vector_time,vettore_thrust,p,opt_CT,opt_ST,vettore_velocities,vettore_acceleration);

hold off 
end

