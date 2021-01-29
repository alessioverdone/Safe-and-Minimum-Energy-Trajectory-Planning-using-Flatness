clear
syms tend
tic

%Set start and final point in the usage function which runs the rrt method and returns the path
obstacle = 'no_obstacles.txt';
p = 0.2;

path = RRT(obstacle);

%Test repeated points
dim=size(path);
path_temp=[];
for row=1:dim(1)
    if row ~= dim(1)
        if (norm(path(row,:) - path(row+1,:))) > 2.2
            disp((norm(path(row,:) - path(row+1,:))))
            path_temp = [path_temp ; path(row,:)];
        else
            continue
        end
    else
        path_temp = [path_temp ; path(row,:)];
    end
        
end

path=path_temp;
disp(path)
len_row = size(path);
stringa = ['Total iterations : ', num2str(len_row(1)-1)];
disp(stringa)

%Start optimization process for each section of the path
list_time=[];

%Optimization problem parameters
lb =0.01;
A = [];
b=[];
Aeq= [];
beq=[];
x0 = 2.5;
ub=18;
%Options parameters
options = optimoptions('fmincon','Display','iter','Algorithm','active-set');


%%%%%%%%%%%%%%%%%%%%%%% DEFINE TEST PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

options.MaxFunctionEvaluations =30;
options.ConstraintTolerance =1e-10;
options.StepTolerance =0;
opt_CT = options.ConstraintTolerance;
opt_ST = options.StepTolerance;
opt_MFE = options.MaxFunctionEvaluations;


%Flag:0 inizio; 1 mezzo; 2 fine
passaggio=0;
fin2=size(path);
fin = fin2(1)-1;
for i=1:(len_row(1)-1)
    switch i
        case 1
            passaggio = 0;
        case (len_row(1)-1)
            passaggio = 2;
        otherwise
            passaggio = 1;
    end
    if (i == 1) && (i ==(fin))
        passaggio = 3;
    end
    disp(['Iteration number :',num2str(i)])
    disp(passaggio)
    init_point2 = path(i,:);
    init_point = [init_point2(1) init_point2(2) init_point2(3) 0];
    final_point2 = path(i+1,:);
    final_point = [final_point2(1) final_point2(2) final_point2(3) 0];
    fun = @(tf) double(Function_obj(tf,init_point,final_point,passaggio));
    nonlcon = @(tf) Function_cond(tf,init_point,final_point,p,passaggio);
    disp('Computing...')
    tf = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    list_time = [ list_time tf];
    disp(list_time)%19.06
end
tempo_ex =toc;
disp(list_time)

%Build the trajectory point by point
Build_trajectory(path,list_time,p,opt_CT,opt_ST,obstacle);
final_energy = 0;
final_time = sum(list_time);

%Compute total time and energy
for w=1:size(path)-1
    switch w
        case 1
            passaggio = 0;
        case size(path)-1
            passaggio = 2;
        otherwise
            passaggio = 1;
    end
    if (w == 1) && (w ==(fin))
        passaggio = 3;
    end
    init_point2 = path(w,:);
    init_point = [init_point2(1) init_point2(2) init_point2(3) 0];
    final_point2 = path(w+1,:);
    final_point = [final_point2(1) final_point2(2) final_point2(3) 0];
    energy = Function_obj(list_time(w),init_point,final_point,passaggio);
    disp(energy)
    final_energy = final_energy + energy;
end

disp(' Time of the optimization execution')
disp(tempo_ex)
disp('Total time of the trajectory')
disp(final_time)
disp('Total energy employeed')
disp(final_energy)






















