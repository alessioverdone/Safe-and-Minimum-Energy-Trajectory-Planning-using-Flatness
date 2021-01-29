function plot_thrust(list_time,vettore_thrust2,p,opt_CT,opt_ST,vettore_velocities2,vettore_acceleration2)
    syms t

    q = vettore_thrust2;
    v1 = vettore_velocities2;
    a1 = vettore_acceleration2;
    
    vec_x=[0];
    vec_y=[0];
    vec_z=[0];
    vec_4=[0];
    vec_vx=[0];
    vec_vy=[0];
    vec_vz=[0];
    vec_v4=[0];
    vec_ax=[0];
    vec_ay=[0];
    vec_az=[0];
    vec_a4=[0];
    c = size(q);
    c = c(2);
    x_space = [-0.1];
    for i=1:4:c
        qx = q(i);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointx = subs(qx,t,j);
            vec_x = [vec_x pointx];
            x_len = size(x_space);
            x_len = x_len(2);
            x_elem=(x_space(x_len)+0.1);
            x_space = [x_space  x_elem];
        end
        qy = q(i+1);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointy = subs(qy,t,j);
            vec_y = [vec_y pointy];
        end
        qz = q(i+2);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointz = subs(qz,t,j);
            vec_z = [vec_z pointz];
        end
        q4 = q(i+3);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            point4 = subs(q4,t,j);
            vec_4 = [vec_4 point4];
        end
        %V1
        v1x = v1(i);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointvx = subs(v1x,t,j);
            vec_vx = [vec_vx pointvx];
        end
        %V2
        v1y = v1(i+1);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointvy = subs(v1y,t,j);
            vec_vy = [vec_vy pointvy];
        end
        %V3
        v1z = v1(i+2);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointvz = subs(v1z,t,j);
            vec_vz = [vec_vz pointvz];
        end
        %V4
        v14 = v1(i+3);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointv4 = subs(v14,t,j);
            vec_v4 = [vec_v4 pointv4];
        end
        %A1
        a1x = a1(i);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointax = subs(a1x,t,j);
            vec_ax = [vec_ax pointax];
        end
        %A2
        a1y = a1(i+1);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointay = subs(a1y,t,j);
            vec_ay = [vec_ay pointay];
        end
        %A3
        a1z = a1(i+2);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointaz = subs(a1z,t,j);
            vec_az = [vec_az pointaz];
        end
        %A4
        a14 = a1(i+3);
        for j=0:0.1:(list_time((i+3)/4)-mod(list_time((i+3)/4),0.1))
            pointa4 = subs(a14,t,j);
            vec_a4 = [vec_a4 pointa4];
        end
    end
    
    x_len = size(x_space);
    x_len = x_len(2);
    max_thrust = 1;
    max_thrust_vec = max_thrust+zeros(1,x_len);
    max_prob_thrust = p+zeros(1,x_len);
    max_prob_thrust2 = (1-p)+zeros(1,x_len);
    
    %Plot single thrusts
    sing_t = figure();
    subplot(4,1,1)
    plot(x_space,vec_x,x_space,max_thrust_vec,'b',x_space,max_prob_thrust,'--',x_space,max_prob_thrust2,'--')
    axis([0 x_len/10 0 1.5])
    title('Tx')
    xlabel('Time[sec]')
    ylabel('Norm. thrust')
    subplot(4,1,2)
    plot(x_space,vec_y,x_space,max_thrust_vec,'b',x_space,max_prob_thrust,'--',x_space,max_prob_thrust2,'--')
    axis([0 x_len/10 0 1.5])
    title('Ty')
    xlabel('Time[sec]')
    ylabel('Norm. thrust')
    subplot(4,1,3)
    plot(x_space,vec_z,x_space,max_thrust_vec,'b',x_space,max_prob_thrust,'--',x_space,max_prob_thrust2,'--')
    axis([0 x_len/10 0 1.5])
    title('Tz')
    xlabel('Time[sec]')
    ylabel('Norm. thrust')
    subplot(4,1,4)
    plot(x_space,vec_4,x_space,max_thrust_vec,'b',x_space,max_prob_thrust,'--',x_space,max_prob_thrust2,'--')
    axis([0 x_len/10 0 1.5])
    title('T4')
    xlabel('Time[sec]')
    ylabel('Norm. thrust')
    %saveas(sing_t,['Images_exp/Prove_finali/4_thrust_p=',num2str(p),'_ConTol=',num2str(opt_CT),'_Step_tol=',num2str(opt_ST),'.png'])
    
    %Plot thrust toghether
    four_t = figure();
    plot(x_space,vec_x,x_space,vec_y,x_space,vec_z,x_space,vec_4,x_space,max_thrust_vec,'b',x_space,max_prob_thrust,'--',x_space,max_prob_thrust2,'--')
    axis([0 x_len/10 0 1.1])
    title('Thrusts')
    xlabel('Time[sec]')
    ylabel('Norm. thrust')
    %saveas(four_t,['Images_exp/Prove_finali/Thrust_together_p=',num2str(p),'_ConTol=',num2str(opt_CT),'_Step_tol=',num2str(opt_ST),'.png'])
    
    %Plot sum thrust
    sum_t = figure();
    plot(x_space,vec_x+vec_y+vec_z+vec_4)
    axis([0 x_len/10 0 3])
    title('Total Thrust')
    xlabel('Time[sec]')
    ylabel('Norm. thrust')
    %saveas(sum_t,['Images_exp/Prove_finali/Tot_sum_p=',num2str(p),'_ConTol=',num2str(opt_CT),'_Step_tol=',num2str(opt_ST),'.png'])
    
    %Plot velocity
    vec_fig = figure();
    plot(x_space,vec_vx,x_space,vec_vy,x_space,vec_vz)
    min_vy = double(min([vec_vx,vec_vy,vec_vz]));
    max_vy = double(max([vec_vx,vec_vy,vec_vz]));
    disp(min_vy)
    disp(max_vy)
    axis([0 x_len/10 (min_vy-0.5) (max_vy+0.5)])
    title('Velocity')
    xlabel('Time [sec]')
    ylabel('Velocity [m/sec]')
    %saveas(vec_fig,['Images_exp/Prove_finali/Velocity_p=',num2str(p),'_ConTol=',num2str(opt_CT),'_Step_tol=',num2str(opt_ST),'.png'])

    %Plot accelleration
    acc_fig = figure();
    plot(x_space,vec_ax,x_space,vec_ay,x_space,vec_az)
    min_ay = double(min([vec_ax,vec_ay,vec_az]));
    max_ay = double(max([vec_ax,vec_ay,vec_az]));
    disp(min_ay)
    disp(max_ay)
    axis([0 x_len/10 (min_ay-0.5) (max_ay+0.5)])
    title('Acceleration')
    xlabel('Time [sec]')
    ylabel('Acceleration [m/sec^2]')
    %saveas(acc_fig,['Images_exp/Prove_finali/Acceleration_p=',num2str(p),'_ConTol=',num2str(opt_CT),'_Step_tol=',num2str(opt_ST),'.png'])

    
   
end