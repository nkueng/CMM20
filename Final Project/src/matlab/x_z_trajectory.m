clear all, close all, clc;

syms x_0 x_f t_f z_0 z_f

A = [t_f^5, t_f^4, t_f^3;
    5*t_f^4, 4*t_f^3, 3*t_f^2;
    20*t_f^3, 12*t_f^2, 6*t_f];

b = [x_f - x_0;
    0;
    0];

params = A\b;

% coeffs = subs(params, [x_0, x_f, t_f], [0, 1, 1])
z_params = subs(params, [x_0, x_f], [z_0, z_f]);

% plots
x_0 = 0;
x_f = .05;
t_f = 1;
z_0 = 0;
z_f = 3;
subs(A)
subs(b)
x_params = subs(params)
z_params = subs(z_params);
t = linspace(0, t_f, 100);
x = x_params(1) * t.^5 + x_params(2) * t.^4 + x_params(3) * t.^3 + x_0;
z = z_params(1) * t.^5 + z_params(2) * t.^4 + z_params(3) * t.^3 + z_0;
x_dot = 5 * x_params(1) * t.^4 + 4 * x_params(2) * t.^3 + 3 * x_params(3) * t.^2;
x_ddot = 20 * x_params(1) * t.^3 + 12 * x_params(2) * t.^2 + 6 * x_params(3) * t.^1;
figure 
plot(t, x)
xlabel('t')
ylabel('$x(t)$', 'Interpreter','latex')

figure 
plot(t, x_dot)
xlabel('t')
ylabel('$\dot{x}(t)$', 'Interpreter','latex')

figure 
plot(t, x_ddot)
xlabel('t')
ylabel('$\ddot{x}(t)$', 'Interpreter','latex')

figure 
plot(x, z)
xlabel('x')
ylabel('z')