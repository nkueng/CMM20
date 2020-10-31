syms phi_max h_max h_min
h_ref = 0.12;

A = [  phi_max^7,    phi_max^6,   phi_max^5,   phi_max^4,   phi_max^3;
               1,            1,           1,           1,           1;
               7,            6,           5,           4,           3;
     7*phi_max^6,  6*phi_max^5, 5*phi_max^4, 4*phi_max^3, 3*phi_max^2;
              42,           30,          20,          12,           6];

b = [h_max;
     -h_min;
     0;
     0;
     0];


param = inv(A) * b;

coefficientsNumerical = subs(param, [phi_max, h_max, h_min], [0.5, 1/5*h_ref, 0])
figure
x = linspace(0, 1, 100);
y = coefficientsNumerical(1) * x.^7 + coefficientsNumerical(2) * x.^6 + coefficientsNumerical(3) * x.^5 + coefficientsNumerical(4) * x.^4 + coefficientsNumerical(5) * x.^3;
y_prime = 7 * coefficientsNumerical(1) * x.^6 + 6 * coefficientsNumerical(2) * x.^5 + 5 * coefficientsNumerical(3) * x.^4 + 4 * coefficientsNumerical(4) * x.^3 + 3 * coefficientsNumerical(5) * x.^2;
y_pprime = 42 * coefficientsNumerical(1) * x.^5 + 30 * coefficientsNumerical(2) * x.^4 + 20 * coefficientsNumerical(3) * x.^3 + 12 * coefficientsNumerical(4) * x.^2 + 6 * coefficientsNumerical(5) * x;
y_ppprime = 210 * coefficientsNumerical(1) * x.^4 + 120 * coefficientsNumerical(2) * x.^3 + 60 * coefficientsNumerical(3) *  x.^2 + 24 * coefficientsNumerical(4) * x + 6 * coefficientsNumerical(5);
plot(x, y, 'DisplayName','position')
hold on
figure
plot(x, y_prime, 'DisplayName','velocity')
figure
plot(x, y_pprime, 'DisplayName','acceleration')
figure
plot(x, y_ppprime)
xlabel('t')
ylabel('h_{leg}(t)')