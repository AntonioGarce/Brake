phi_s0 = 0; %initial entrance angle(°)
n_s0 = 0; %initial disc speed(/min)
delta_t = 1e-6; %calculation increment(s)
t_end = 0.1; %calculation time(s)
n_in = 0.05; %input speed(/min)

%convert units
phi_s0 = phi_s0*pi/180; %initial entrance angle(°)
n_s0 = n_s0*pi/180; %initial disc speed(/min)

x0 = [phi_s0,n_s0];
tspan = 0:delta_t:t_end;

[t,x] = ode45(@odefun, tspan, x0);
phi_in = n_in*360/60*t; %entrance angle[°]
phi_s = x(:,1)*180/pi; % disc angle[°]
n_s = x(:,2)*60/(2*pi);
yyaxis left
plot(t,phi_s,'k-',t,phi_in,'r-')
ylabel('entrance and disc angle[°]')
yyaxis right
plot(t,n_s,'b-');
ylabel('disc angle speed[r/min]')
xlabel('time[s]')
legend({'disc angle','entrance angle','disc angle speed'},'location','southoutside','NumColumns',3)

function dxdt = odefun(t,x)
    C = 5236; %Substitute torsional stiffness(Nm/°)
    J = 0.122; %equivalent moment of inertia(kg?m²)
    mu_s = 0.54; %coefficient of static friction
    mu_b = 0.49; %coefficient of sliding friction
    d_r = 0.3125; %reaming diameter(m)
    Fspinn = 200; %resilience(N)
    n_in = 0.05; %input speed(/min)
    dn = 5e-5; %speed range static friction(/min)
    
    %convert the units to standard uints
    C = C*180/pi; %(Nm/rad)
    n_in = n_in*2*pi/60; %(rad/s)
    dn = dn*2*pi/60; % (rad/s)
    
    dxdt = zeros(2,1);
    dxdt(1) = x(2);
    
    phi_in = n_in*t;
    T_e = C*(phi_in-x(1));
    F_e = T_e/d_r;
    F_c = mu_b*Fspinn;
    F_s = mu_s*Fspinn;
    
    if (abs(x(2))>=dn)
        F_r = sign(x(2))*F_c;
    else
        F_r = sign(F_e)*min(abs(F_e),F_s);
    end
    dxdt(2) = (T_e - F_r*d_r)/J;
end


    
