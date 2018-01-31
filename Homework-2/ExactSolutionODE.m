
clear all; clc;

syms m k c real
assumeAlso(m > 0);
assumeAlso(k > 0);
assumeAlso(c > 0);

syms x(t)
oms = sqrt(k/m);
syms omega real


xp = diff(x, t);
xpp = diff(xp, t);

Eq = m*xpp + c*xp + k*x;

syms A real
solution_damped = dsolve(Eq == 0, [xp(0) == 0, x(0) == A] );
disp(solution_damped)

syms cc zeta real

solution_damped_rewrite = simplify(expand(subs(subs(simplify(subs(subs(solution_damped, 4*k*m, cc*cc), c, zeta*cc)),cc,2*sqrt(k*m)),sqrt(k/m),omega)));
disp(solution_damped_rewrite)

figure
fplot(subs(solution_damped_rewrite, [A, omega, zeta], [-1, 2, 0.1]), [0, 10])
grid on