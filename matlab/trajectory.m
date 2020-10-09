% checking the math for the trapezoidal trajectory

amax = 2;
vmax = 2;
p0 = 0;
p3 = 0;
v0 = 0;
v3 = 0;
t3d = t3+.001;

v0 = min(max(v0,-vmax),vmax);
v3 = min(max(v3,-vmax),vmax);
dp = p3-p0;
dv = v3-v0;

%roots([amax 2*v0 -dp-v0*dv/amax-dv^2/2/amax])
%roots([-amax 2*v0 -dp+v0*dv/amax+dv^2/2/amax])

c = struct;
c.v0 = v0;
c.p0 = p0;
t3sol = inf;
for a = [-amax,amax]
    s = 4*v0^2 + 4*(dp*a+(v0+v3)*dv/2);
    if (s > 0)
        for q = [-1,1]        
            t1 = (-2*v0 + q*sqrt(s))/2/a;
            if (t1 > 0)
                t3 = 2*t1-dv/a;
                if (t3 > 0 && t3 > t1)
                    if (t3 < t3sol)
                        t3sol = t3;
                        c.t1 = t1;
                        c.t3 = t3;
                        c.a1 = a;
                        c.a2 = -a;
                    end
                end
            end
        end
    end
end

if (abs(c.v0+c.a1*c.t1) < vmax)
    % pyramid velocity
    c.t2 = c.t1;
else
    % trapezoidal velocity
    q = sign(c.v0+c.a1*c.t1);
    vsat = q*vmax;
    tp = c.t1;
    c.t1 = (vsat - v0)/c.a1; 
    % the distance traveled in the original trajectory when v was overspeed
    xtrap = 2*(tp-c.t1)*(vsat+.5*c.a1*(tp-c.t1));
    c.t2 = c.t1 + xtrap/vsat;
    c.t3 = c.t3-2*tp+c.t1+c.t2;
end

c

% recompute for time, keeping a at amax, but now can have
% various combinations of amax
% I think it's guaranteed to be trapezoid given that
% could also consider the minimum acceleration pyramid
% or other combinations
if (c.t3 <= t3d)
    t3 = t3d;
    c.t3 = t3;
    for a1 = [-amax,amax]
        for a2 = [-amax,amax]
            a = .5*a1*(-1 + a1/a2);
            b = a1*(t3 - dv/a2);
            cc = v0*t3 + .5*dv^2/a2 - dp;
            sol = quadratic_positive(a,b,cc);
            for t1 = sol'
                if (t1 > 0)
                    t2 = -(dv-a1*t1)/a2+t3;
                    if (t2 >= t1+eps(t1) && t2 <= t3)
                        disp('sol')
                        c.t1 = t1;
                        c.t2 = t2;
                        c.a1 = a1;
                        c.a2 = a2;
                    end
                end
            end
        end
    end
end

c

time = (0:.001:c.t3+.1)';
x = zeros(size(time));
v = zeros(size(time));
a = zeros(size(time));

for i = 1:length(time)
    t = time(i);
    if (t > c.t3)
        t = c.t3;
    end
    if (t < c.t1)
        x(i) = c.p0 + c.v0*t + .5*c.a1*t*t;
        v(i) = c.v0 + c.a1*t;
        a(i) = c.a1;
    elseif (t < c.t2) 
        x(i) = c.p0 + c.v0*c.t1 + .5*c.a1*c.t1*c.t1 + (c.v0 + c.a1*c.t1)*(t-c.t1);
        v(i) = (c.v0+c.a1*c.t1);
        a(i) = 0;
    else
        x(i) = c.p0 + c.v0*c.t1 + .5*c.a1*c.t1*c.t1 + (c.v0 + c.a1*c.t1)*(t-c.t1) + .5*c.a2*(t-c.t2)*(t-c.t2);
        v(i) = (c.v0+c.a1*c.t1) + c.a2*(t-c.t2);
        a(i) = c.a2;
    end
end
v2 = [0;diff(x)./diff(time)];
a2 = [0;diff(v)./diff(time)];

figure(1); clf;
subplot(3,1,1);
plot(time,x);
subplot(3,1,2);
plot(time,[v,v2]);
subplot(3,1,3);
plot(time,[a,a2]);
ylim(1.5*[-abs(c.a1) abs(c.a1)]);
