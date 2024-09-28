function dxdt = MTAHVStateFcn(x,u)
% States x1 = [ lateral velocity (v)
%               yaw rate (phi1_dot)
%               first articulation angle (theta1)
%               first articulation angle rate (theta1_dot)
%               second articulation angle (theta2)
%               second articulation angle rate (theta2_dot)
%               third articulation angle (theta3)
%               third articulation angle rate (theta3_dot)
%               longitudinal velocity (u)];
%
% Inputs u = [   steering angle (delta)
%                longitudinal force (Fx1r)];
%
% Outputs:
%     dxdt = state derivatives

%% Vehicle Parameters
format long
m1 = 6310;   % Mass of the first unit
m2 = 24690;   % Mass of the second unit
m3 = 4354;   % Mass of the third unit
m4 = 24690;    % Mass of the forth unit
j1 = 19665;   % Moment of inertia about Z axis of the first unit
j2 = 274658;   % Moment of inertia about Z axis of the second unit
j3 = 1990;   % Moment of inertia about Z axis of the third unit
j4 = 274658;   % Moment of inertia about Z axis of the forth unit
a1 = 3.385;    % Distance between COG to the front axle of the first unit
a2 = 6.485;    % Distance between COG to the fifth wheel of the first unit
a3 = 1.489;    % Distance between COG to the fifth wheel of the second unit
a4 = 6.485;    % Distance between COG to the fifth wheel of the third unit
b1 = 4.25;    % Distance between COG to the rear axle of the first unit
b2 = 6.015;    % Distance between COG to the rear axle of the second unit
b3 = 0;    % Distance between COG to the rear axle of the third unit
b4 = 6.015;    % Distance between COG to the rear axle of the forth unit
c1 = 4.25;    % Distance between COG to the fifth wheel of the first unit
c2 = 8.8;    % Distance between COG to the fifth wheel of the second unit
c3 = 0;    % Distance between COG to the fifth wheel of the third unit
cs1f = 286487; % Cornering stiffness of the front tires for the first unit (N/rad)
cs1r = 814206; % Cornering stiffness of the rear tires for the first unit(N/rad)
cs2r = 789054; % Cornering stiffness of the rear tires for the second unit(N/rad)
cs3r = 862206; % Cornering stiffness of the rear tires for the third unit(N/rad)
cs4r = 789054; % Cornering stiffness of the rear tires for the forth unit(N/rad)

%% State Equations #1
% Order of writing equations  = [  Q_v
%                                  Q_phi1
%                                  Q_theta1
%                                  Q_theta2
%                                  Q_theta3
%                                  dtheta1
%                                  dtheta2
%                                  dtheta3
%                                  Q_u];

% M*X_dot = A*X + B*u 

% M matrix
M = [m1+m2+m3+m4, -((a3+c2)*m3+(a3+a4+c2+c3)*m4+a2*(m2+m3+m4)+c1*(m2+m3+m4)), 0, -((a3+c2)*m3+(a3+a4+c2+c3)*m4+a2*(m2+m3+m4)), 0, -(a3*m3+(a3+a4+c3)*m4), 0, -a4*m4, 0;
    -(a2+c1)*m2-(a2+a3+c1+c2)*m3-(a2+a3+a4+c1+c2+c3)*m4, j1+j2+j3+j4+((a2+c1)^2)*m2+((a2+a3+c1+c2)^2)*m3+((a2+a3+a4+c1+c2+c3)^2)*m4, 0,j2+j3+j4+a2*(a2+c1)*m2+(a2+a3+c2)*(a2+a3+c1+c2)*m3+(a2+a3+a4+c2+c3)*(a2+a3+a4+c1+c2+c3)*m4, 0, j3+j4+a3*(a2+a3+c1+c2)*m3+(a3+a4+c3)*(a2+a3+a4+c1+c2+c3)*m4, 0, j4+a4*(a2+a3+a4+c1+c2+c3)*m4, 0;
     (-a2*m2-(a2+a3+c2)*m3-(a2+a3+a4+c2+c3)*m4), (j2+j3+j4+a2*(a2+c1)*m2+(a2+a3+c2)*(a2+a3+c1+c2)*m3+(a2+a3+a4+c2+c3)*(a2+a3+a4+c1+c2+c3)*m4), 0, j2+j3+j4+(a2^2)*m2+((a2+a3+c2)^2)*m3+((a2+a3+a4+c2+c3)^2)*m4, 0, j3+j4+a3*(a2+a3+c2)*m3+(a3+a4+c3)*(a2+a3+a4+c2+c3)*m4, 0, j4+a4*(a2+a3+a4+c2+c3)*m4, 0;
    -a3*m3-(a3+a4+c3)*m4, (j3+j4+a3*(a2+a3+c1+c2)*m3+(a3+a4+c3)*(a2+a3+a4+c1+c2+c3)*m4), 0, (j3+j4+a3*(a2+a3+c2)*m3+(a3+a4+c3)*(a2+a3+a4+c2+c3)*m4), 0, j3+j4+(a3^2)*m3+((a3+a4+c3)^2)*m4, 0, j4+a4*(a3+a4+c3)*m4, 0;
    -a4*m4, (j4+a4*(a2+a3+a4+c1+c2+c3)*m4), 0, j4+a4*(a2+a3+a4+c2+c3)*m4, 0, (j4+a4*(a3+a4+c3)*m4), 0, j4+(a4^2)*m4, 0;
     0, 0, 1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, m1+m2+m3+m4];

a11 = - (cs1f + cs1r + cs2r + cs3r + cs4r)/x(9);
a12 = (-a1*cs1f + b1*cs1r + (a2 + b2 + c1)*cs2r + (a2 + a3 + b3 + c1 + c2)*cs3r ...
     +(a2 + a3 + a4 + b4 + c1 + c2 + c3)*cs4r - m1*(x(9)^2) - m2*(x(9)^2) - m3*(x(9)^2) - m4*(x(9)^2))/x(9);
a13 = cs2r + cs3r + cs4r;
a14 = ((a2 +b2)*cs2r + (a2 + a3 + b3 + c2)*cs3r + (a2 + a3 + a4 + b4 + c2 + c3)*cs4r)/x(9);
a15 = cs3r + cs4r;
a16 = ((a3 + b3)*cs3r + (a3 + a4 + b4 + c3)*cs4r)/x(9);
a17 = cs4r;
a18 = ((a4 + b4)*cs4r)/x(9);
a19 = 0;

a21 = (-a1*cs1f + b1*cs1r + (a2 + b2 + c1)*cs2r + (a2 + a3 + b3 + c1 + c2)*cs3r + (a2 + a3 + a4 + b4 + c1 + c2 + c3)*cs4r)/x(9);
a22 = - ((a1^2*cs1f + b1^2*cs1r + a2^2*cs2r + 2*a2*b2*cs2r + b2^2*cs2r + 2*a2*c1*cs2r + 2*b2*c1*cs2r + c1^2*cs2r + a2^2*cs3r + 2*a2*a3*cs3r + a3^2*cs3r + 2*a2*b3*cs3r + 2*a3*b3*cs3r + b3^2*cs3r ...
      + 2*a2*c1*cs3r + 2*a3*c1*cs3r + 2*b3*c1*cs3r + c1^2*cs3r + 2*a2*c2*cs3r + 2*a3*c2*cs3r + 2*b3*c2*cs3r + 2*c1*c2*cs3r + c2^2*cs3r ...
      + a2^2*cs4r + 2*a2*a3*cs4r + a3^2*cs4r + 2*a2*a4*cs4r + 2*a3*a4*cs4r + a4^2*cs4r + 2*a2*b4*cs4r + 2*a3*b4*cs4r + 2*a4*b4*cs4r + b4^2*cs4r + 2*a2*c1*cs4r + 2*a3*c1*cs4r + 2*a4*c1*cs4r + 2*b4*c1*cs4r + c1^2*cs4r ...
      + 2*a2*c2*cs4r + 2*a3*c2*cs4r + 2*a4*c2*cs4r + 2*b4*c2*cs4r + 2*c1*c2*cs4r + c2^2*cs4r + 2*a2*c3*cs4r + 2*a3*c3*cs4r + 2*a4*c3*cs4r + 2*b4*c3*cs4r + 2*c1*c3*cs4r + 2*c2*c3*cs4r + c3^2*cs4r - a2*m2*x(9)^2 - c1*m2*x(9)^2 -a2*m3*x(9)^2 - a3*m3*x(9)^2 - c1*m3*x(9)^2 ...
      - c2*m3*x(9)^2 - a2*m4*x(9)^2 - a3*m4*x(9)^2 - a4*m4*x(9)^2 -c1*m4*x(9)^2 - c2*m4*x(9)^2 - c3*m4*x(9)^2)/x(9));

a23 = - b2*cs2r - c1*cs2r - a3*cs3r - b3*cs3r - c1*cs3r - c2*cs3r - a3*cs4r - a4*cs4r - b4*cs4r - c1*cs4r - c2*cs4r - c3*cs4r - a2*(cs2r + cs3r + cs4r);
a24 = - ((b2^2*cs2r + b2*c1*cs2r + a3^2*cs3r + 2*a3*b3*cs3r + b3^2*cs3r + a3*c1*cs3r + b3*c1*cs3r + 2*a3*c2*cs3r + 2*b3*c2*cs3r + c1*c2*cs3r + c2^2*cs3r ...
      + a3^2*cs4r + 2*a3*a4*cs4r + a4^2*cs4r + 2*a3*b4*cs4r + 2*a4*b4*cs4r + b4^2*cs4r + a3*c1*cs4r + a4*c1*cs4r + b4*c1*cs4r + 2*a3*c2*cs4r + 2*a4*c2*cs4r ...
      + 2*b4*c2*cs4r + c1*c2*cs4r + c2^2*cs4r + 2*a3*c3*cs4r + 2*a4*c3*cs4r + 2*b4*c3*cs4r + c1*c3*cs4r + 2*c2*c3*cs4r + c3^2*cs4r + a2^2*(cs2r + cs3r + cs4r) ...
      + a2*(2*b2*cs2r + c1*(cs2r + cs3r + cs4r) + 2*(b3*cs3r + c2*cs3r + a4*cs4r + b4*cs4r + c2*cs4r + c3*cs4r + a3*(cs3r + cs4r))))/x(9));
a25 = - b3*cs3r - c1*cs3r - c2*cs3r - a4*cs4r - b4*cs4r - c1*cs4r - c2*cs4r - c3*cs4r - a2*(cs3r + cs4r) - a3*(cs3r + cs4r);
a26 = - ((b3^2*cs3r + b3*c1*cs3r + b3*c2*cs3r + a4^2*cs4r + 2*a4*b4*cs4r + b4^2*cs4r + a4*c1*cs4r + b4*c1*cs4r + a4*c2*cs4r + b4*c2*cs4r + 2*a4*c3*cs4r ...
      + 2*b4*c3*cs4r + c1*c3*cs4r + c2*c3*cs4r + c3^2*cs4r + a3^2*(cs3r + cs4r) + a2*(b3*cs3r + (a4 + b4 + c3)*cs4r + a3*(cs3r + cs4r)) ...
      + a3*(2*b3*cs3r + c2*cs3r + 2*a4*cs4r + 2*b4*cs4r + c2*cs4r + 2*c3*cs4r + c1*(cs3r + cs4r)))/x(9));
a27 = -(a2 + a3 + a4 + b4 + c1 + c2 + c3)*cs4r;
a28 = - ((a4 + b4)*(a2 + a3 + a4 + b4 + c1 + c2 + c3)*cs4r)/x(9);
a29 = 0;


a31 = (b2*cs2r + (a3 + b3 + c2)*cs3r + (a3 + a4 + b4 + c2 + c3)*cs4r + a2*(cs2r + cs3r + cs4r))/x(9);
a32 = - ((b2^2*cs2r + b2*c1*cs2r + (a3 + b3)^2*cs3r + a3*c1*cs3r + b3*c1*cs3r + 2*a3*c2*cs3r + 2*b3*c2*cs3r + c1*c2*cs3r + c2^2*cs3r + a3^2*cs4r + 2*a3*a4*cs4r ...
      + a4^2*cs4r + 2*a3*b4*cs4r + 2*a4*b4*cs4r + b4^2*cs4r + a3*c1*cs4r + a4*c1*cs4r + b4*c1*cs4r + 2*a3*c2*cs4r + 2*a4*c2*cs4r + 2*b4*c2*cs4r + c1*c2*cs4r ...
      + c2^2*cs4r + 2*a3*c3*cs4r + 2*a4*c3*cs4r + 2*b4*c3*cs4r + c1*c3*cs4r + 2*c2*c3*cs4r + c3^2*cs4r + a2^2*(cs2r + cs3r + cs4r) - ((a3 + c2)*m3 ...
      + (a3 + a4 + c2 + c3)*m4)*x(9)^2 + a2*(2*b2*cs2r + 2*(a3 + b3 + c2)*cs3r + 2*(a3 + a4 + b4 + c2 + c3)*cs4r + c1*(cs2r + cs3r + cs4r) - (m2 + m3 + m4)*x(9)^2))/x(9));

a33 = - b2*cs2r - a3*cs3r - b3*cs3r - c2*cs3r - a3*cs4r - a4*cs4r - b4*cs4r - c2*cs4r - c3*cs4r - a2*(cs2r + cs3r + cs4r);
a34 = - ((b2^2*cs2r + a3^2*cs3r + 2*a3*b3*cs3r + b3^2*cs3r + 2*a3*c2*cs3r + 2*b3*c2*cs3r + c2^2*cs3r + a3^2*cs4r + 2*a3*a4*cs4r + a4^2*cs4r + 2*a3*b4*cs4r ...
      + 2*a4*b4*cs4r + b4^2*cs4r + 2*a3*c2*cs4r + 2*a4*c2*cs4r + 2*b4*c2*cs4r + c2^2*cs4r + 2*a3*c3*cs4r + 2*a4*c3*cs4r + 2*b4*c3*cs4r + 2*c2*c3*cs4r + c3^2*cs4r ...
      + a2^2*(cs2r + cs3r + cs4r) + 2*a2*(b2*cs2r + b3*cs3r + c2*cs3r + a4*cs4r + b4*cs4r + c2*cs4r + c3*cs4r + a3*(cs3r + cs4r)))/x(9));
a35 = - b3*cs3r - c2*cs3r - a4*cs4r - b4*cs4r - c2*cs4r - c3*cs4r - a2*(cs3r + cs4r) - a3*(cs3r + cs4r);
a36 = -((b3^2*cs3r + b3*c2*cs3r + a4^2*cs4r + 2*a4*b4*cs4r + b4^2*cs4r + a4*c2*cs4r + b4*c2*cs4r + 2*a4*c3*cs4r + 2*b4*c3*cs4r + c2*c3*cs4r + c3^2*cs4r ...
      + a3^2*(cs3r + cs4r) + a2*(b3*cs3r + (a4 + b4 + c3)*cs4r + a3*(cs3r + cs4r)) + a3*(2*b3*cs3r + 2*(a4 + b4 + c3)*cs4r + c2*(cs3r + cs4r)))/x(9));
a37 = - (a2 + a3 + a4 + b4 + c2 + c3)*cs4r;
a38 = - (((a4 + b4)*(a2 + a3 + a4 + b4 + c2 + c3)*cs4r)/x(9));
a39 = 0;

a41 = ((a3 + b3)*cs3r + (a3 + a4 + b4 + c3)*cs4r)/x(9);
a42 = - ((b3^2*cs3r + a2*(a3 + b3)*cs3r + b3*c1*cs3r + b3*c2*cs3r + (a4 + b4)^2*cs4r + a4*c1*cs4r + b4*c1*cs4r + a4*c2*cs4r + b4*c2*cs4r + 2*a4*c3*cs4r ...
      + 2*b4*c3*cs4r + c1*c3*cs4r + c2*c3*cs4r + c3^2*cs4r + a2*(a3 + a4 + b4 + c3)*cs4r + a3^2*(cs3r + cs4r) - (a4 + c3)*m4*x(9)^2 + a3*((2*b3 + c1 + c2)*cs3r + 2*(a4 + b4)*cs4r + (c1 + c2 + 2*c3)*cs4r - (m3 + m4)*x(9)^2))/x(9));
a43 = -b3*cs3r - (a4 + b4 + c3)*cs4r - a3*(cs3r + cs4r);
a44 = -((b3^2*cs3r + b3*c2*cs3r + a4^2*cs4r + 2*a4*b4*cs4r + b4^2*cs4r + a4*c2*cs4r + b4*c2*cs4r + 2*a4*c3*cs4r + 2*b4*c3*cs4r + c2*c3*cs4r + c3^2*cs4r ...
      + a3^2*(cs3r + cs4r) + a2*(b3*cs3r + (a4 + b4 + c3)*cs4r + a3*(cs3r + cs4r)) + a3*(2*b3*cs3r + 2*(a4 + b4 + c3)*cs4r + c2*(cs3r + cs4r)))/x(9));
a45 = -b3*cs3r - (a4 + b4 + c3)*cs4r - a3*(cs3r + cs4r);
a46 = - ((b3^2*cs3r + (a4 + b4 + c3)^2*cs4r + a3^2*(cs3r + cs4r) + 2*a3*(b3*cs3r + (a4 + b4 + c3)*cs4r))/x(9));
a47 = - (a3 + a4 + b4 + c3)*cs4r;
a48 = - (((a4 + b4)*(a3 + a4 + b4 + c3)*cs4r)/x(9));
a49 = 0;

a51 = ((a4 + b4) * cs4r)/x(9);
a52 = - (((a4 + b4) * (a2 + a3 + a4 + b4 + c1 + c2 + c3)*cs4r - a4*m4*x(9)^2)/x(9));
a53 = - (a4 + b4) * cs4r;
a54 = - (((a4 + b4)*(a2 + a3 + a4 + b4 + c2 + c3)*cs4r)/x(9));
a55 = - (a4 + b4)*cs4r;
a56 = - (((a4 + b4)*(a3 + a4 + b4 + c3)*cs4r)/x(9));
a57 = - (a4 + b4)*cs4r;
a58 = - (((a4 + b4)^2*cs4r)/x(9));
a59 = 0;

a61 = 0;
a62 = 0;
a63 = 0;
a64 = 1;
a65 = 0;
a66 = 0;
a67 = 0;
a68 = 0;
a69 = 0;

a71 = 0;
a72 = 0;
a73 = 0;
a74 = 0;
a75 = 0;
a76 = 1;
a77 = 0;
a78 = 0;
a79 = 0;


a81 = 0;
a82 = 0;
a83 = 0;
a84 = 0;
a85 = 0;
a86 = 0;
a87 = 0;
a88 = 1;
a89 = 0;

a91 = 0;
a92 = 0;
a93 = 0;
a94 = 0;
a95 = 0;
a96 = 0;
a97 = 0;
a98 = 0;
a99 = 0;

AA = [a11 , a12 , a13 , a14 , a15 , a16 , a17 , a18 , a19;
     a21 , a22 , a23 , a24 , a25 , a26 , a27 , a28 , a29;
     a31 , a32 , a33 , a34 , a35 , a36 , a37 , a38 , a39;
     a41 , a42 , a43 , a44 , a45 , a46 , a47 , a48 , a49;
     a51 , a52 , a53 , a54 , a55 , a56 , a57 , a58 , a59;
     a61 , a62 , a63 , a64 , a65 , a66 , a67 , a68 , a69;
     a71 , a72 , a73 , a74 , a75 , a76 , a77 , a78 , a79;
     a81 , a82 , a83 , a84 , a85 , a86 , a87 , a88 , a89;
     a91 , a92 , a93 , a94 , a95 , a96 , a97 , a98 , a99];
BB = [cs1f   ,  0;
     a1*cs1f,  0;
     0      ,  0;
     0      ,  0;
     0      ,  0;
     0      ,  0;
     0      ,  0;
     0      ,  0;
     0      ,  1];
AAA = M\AA;
BBB = M\BB;
AAAX = AAA*x(1:9);
BBBu = BBB*u;

% States x2 = [ yaw angle (phi1)
%               X global coordinate (X)
%               Y global coordinate (Y)

%dxdt = x;
dxdt(1) = AAAX(1) + BBBu(1);   
dxdt(2) = AAAX(2) + BBBu(2);
dxdt(3) = AAAX(3) + BBBu(3);
dxdt(4) = AAAX(4) + BBBu(4);
dxdt(5) = AAAX(5) + BBBu(5);
dxdt(6) = AAAX(6) + BBBu(6);
dxdt(7) = AAAX(7) + BBBu(7);
dxdt(8) = AAAX(8) + BBBu(8);
dxdt(9) = AAAX(9) + BBBu(9);
dxdt(10) = x(2);
dxdt(11) = cos(x(10))*x(9) - sin(x(10))*x(1);
dxdt(12) = sin(x(10))*x(9) + cos(x(10))*x(1);

