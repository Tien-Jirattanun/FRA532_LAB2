% -- Property --

I_xx = 0.0347563;
I_yy = 0.07;
I_zz = 0.0977;
m = 1.5;
g = 9.81;

A = zeros(12,12);
A(1,4) = 1; A(2,5) = 1; A(3,6) = 1;
A(4,8) = g; A(5,7) = -g;
A(7,10) = 1; A(8,11) = 1; A(9,12) = 1;

B = zeros(12, 4);
B(6, 1) = 1/m;
B(10, 2) = 1/I_xx; B(11, 3) = 1/I_yy; B(12, 4) = 1/I_zz;


Q = diag([
    30, 30, 30, ...
    5, 5, 5,    ...
    1, 1, 2,    ...
    0.1, 0.1, 0.1,]);

R = diag([1, 2, 2, 2]); %T, T_x, T_y, T_z

[K,S,P] = lqr(A, B, Q, R);

% Run this in MATLAB after calculating K

fprintf('LQR Gain Matrix K (4x12):\n');
fprintf('K << ');
for i = 1:4
    fprintf('    ');
    for j = 1:12
        fprintf('%.8f%s', K(i,j), char( (j<12)*',' + (j==12)*',' ));
        if mod(j,4)==0 && j<12, fprintf(' '); end
    end
    fprintf('.\n');
end

fprintf('Eigen value\n');
A_cl = A - B*K;
poles = eig(A_cl)

