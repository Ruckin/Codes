function [XDOT] = hydrofoil_model(X,U)

%-----------------State and Control Vector-------
%Extract state vector

u=X(1);  % u
v=X(2);  % v
w=X(3);  % w
p=X(4);  % p
q=X(5);  % q
r=X(6);  % r
phi=X(7);  % phi
theta=X(8);  % theta
psi=X(9);  % psi

u1=U(1);  %cg

%-----------------Control Limits/Saturation--------

%-----------------Constant-----------------------
g=9.81; 
rho = 1025;
m=104;  %system total mass
W=m*g;
xg=0;
yg=0;
zg=-0.3;

Ix=3.23*10^4;
Iy=3.64*10^4;
Iz=5.92*10^3;
Ixz=32.4;

% starboard front wing segment
x_ff=0.5;
y_ff=0;
z_ff=0.85;
gamma_ff=deg2rad(0);
alpha_0_ff=0;
S_ff=0.09;
CL_ff=5;
CD_ff=0.1;


% starboard aft wing segment
x_af=-0.5;
y_af=0;
z_af=z_ff;
gamma_af=deg2rad(0);
alpha_0_af=deg2rad(-3);
S_af=0.025;
CL_af=4;
CD_af=0.1;

% strut
x_st=0;
y_st=0;
z_st=0.45+0.4/2;
gamma_st=deg2rad(90);
alpha_0_st=0;
S_st=0.2*0.4;
CL_st=3; %not known
CD_st=0.1;

%-----------------Forces and moments-----------------------
%gravitional forces and moments
G = gravitational (W,xg,yg,zg,phi,theta);

%forward wing
H_ff = Hydrodynamic (u,v,w,p,q,r,alpha_0_ff,x_ff,y_ff,z_ff,gamma_ff,rho,S_ff,CD_ff,CL_ff);

%aft wing
H_aa = Hydrodynamic (u,v,w,p,q,r,alpha_0_af,x_af,y_af,z_af,gamma_af,rho,S_af,CD_af,CL_af);

%strut
H_st = Hydrodynamic (u,v,w,p,q,r,alpha_0_st,x_st,y_st,z_st,gamma_st,rho,S_st,CD_st,CL_st);

A=[m    ,  0  ,  0  ,   0 , m*zg,-m*yg;
   0    ,  m  ,  0  ,-m*zg,  0  , m*xg;
   0    ,  0  ,  m  , m*yg,-m*xg,   0 ; 
   0    ,-m*zg, m*yg,  Ix ,  0  , -Ixz;
   m*zg ,  0  ,-m*xg,  0  ,  Iy ,   0 ;
   -m*yg, m*xg,  0  , -Ixz,   0 ,  Iz ];

B=[-m*w*q+m*v*r+m*xg*(q^2+r^2)-m*yg*p*q-m*zg*p*r + G(1)+H_ff(1)+H_aa(1)+H_st(1);
   -m*u*r+m*w*q+m*yg*(r^2+p^2)-m*zg*q*r-m*xg*q*p + G(2)+H_ff(2)+H_aa(2)+H_st(2);
   -m*v*p+m*u*q+m*zg*(p^2+q^2)-m*xg*r*p-m*yg*r*q + G(3)+H_ff(3)+H_aa(3)+H_st(3);
   Ixz*p*q-(Iz-Iy)*q*r-m*yg*(v*p-u*q)+m*zg*(u*r-w*p) + G(4)+H_ff(4)+H_aa(4)+H_st(4);
   -Ixz*(p^2-r^2)-(Ix-Iz)*r*p-m*zg*(w*p-r*v)+m*xg*(v*p-u*q) + G(5)+H_ff(5)+H_aa(5)+H_st(5);
   -Ixz*r*q-(Iy-Ix)*p*q-m*xg*(u*r-w*p)+m*yg*(w*p-r*v)] + G(6)+H_ff(6)+H_aa(6)+H_st(6);



% cosTheta	=	cos(theta);
% if abs(cosTheta)	<=	0.00001
% cosTheta	=	0.00001*sign(cos(theta));
% end

cosTheta = cos(min(max(theta, -pi/2 + 1e-6), pi/2 - 1e-6)); 

xdot1_6 = GJEM(A,B);
xdot1_6(1)=0;
% xdot7_9   = [p + (q*sin(phi) + r*cos(phi))*sin(theta)/cosTheta;
%               (q*cos(phi) - r*sin(phi));
%               (r*cos(phi) + q*sin(phi))/cosTheta];

xdot7_9   = [p;
             q;
             r];


XDOT=[xdot1_6;xdot7_9];

%-----------------Functions-----------------------
    function C=transformation_matrix (alpha, beta, gamma)
        C=[             cos(beta)*cos(alpha)                     ,          -sin(beta)*cos(alpha)                      , -sin(alpha);
           sin(beta)*cos(gamma)-cos(beta)*sin(alpha)*sin(gamma) ,cos(beta)*cos(gamma)+sin(beta)*sin(alpha)*sin(gamma) ,-cos(alpha)*sin(gamma);
           sin(beta)*sin(gamma)+cos(beta)*sin(alpha)*cos(gamma) ,cos(beta)*sin(gamma)-sin(beta)*sin(alpha)*cos(gamma) , cos(alpha)*cos(gamma)];
    end

    function G = gravitational (W,xg,yg,zg,phi,theta)
        G=[ W*sin(theta);
           -W*cos(theta)*sin(phi);
           -W*cos(theta)*cos(phi);
           -yg*W*cos(theta)*cos(phi)+zg*W*cos(theta)*sin(phi);
            zg*W*sin(theta)+xg*W*cos(theta)*cos(phi);
           -xg*W*cos(theta)*sin(phi)-yg*W*cos(theta)*cos(phi)];
    end 

    function H = Hydrodynamic (u,v,w,p,q,r,alpha_0,xf,yf,zf,gamma,rho,S,CD,CL)
        
        V=(u^2+v^2+w^2)^0.5;
        alpha=alpha_0+((w+p*yf-q*xf)*cos(gamma)-(v+r*xf-p*zf)*sin(gamma))/u;
        beta = ((v+r*xf-p*zf)*cos(gamma)+(w+p*yf-q*xf)*sin(gamma))/u;

        C=transformation_matrix (alpha, beta, gamma);
       
        F_v=[-rho*V^2*S*CD;0;-rho*V^2*S*CL*alpha];
        F_F=C*F_v;
        Q_F=[yf*F_F(1)-zf*F_F(2);zf*F_F(1)-xf*F_F(3);xf*F_F(2)-yf*F_F(1)];
        H=[F_F(1);F_F(2);F_F(3);Q_F(1);Q_F(2);Q_F(3)];
        
    end 

    function F = GJEM(A,B)
        
                    %-------------------------------------------------%
                    %          Gauss Jordan Elimination Method        %
                    %               ~Jagadeesh Korukonda~             %
                    %-------------------------------------------------%
    % ===================================================================================
    % ============================ Solving Linear equations =============================
    % ===================================================================================
    % Input: gjem(A,B)
    % System AX = B;
    % A is n-by-n matrix and B is n-by-1 matrix
    % gjem(A,B) will return a n-by-1 matrix(X) which is solution to the System AX = B
    %===================================================================================
    %================================= Matrix inversion ================================
    %===================================================================================
    % Input: gjem(A,B)
    % A is n-by-n matrix and B is n-by-n Identity matrix
    % gjem(A,B) will return a n-by-n matrix(X) which is inverse of Matrix A
    %===================================================================================
    %===================================================================================
    % Main Code
        % clc
        AB = [A,B];                     %%%Augemented Matrix
        [nr,nc] = size(AB);
        j = 1;k=1;
        while(j <= nr && k <= nc)
            [r,c] = find(AB(j:end,k:end),1);    %returns the position of 1st non-zero element
            if ~isempty(r) && ~isempty(c)
                if c>1
                    k = k+c-1;
                end
                % swap
                if r>1
                    AB([r,j],:) = AB([j,r],:);  %swap rows r and j
                end
                % normalize
                AB(j,:) = AB(j,:)/AB(j,k);
                           
                   rows = zeros(1,nr-1);
                   z = 1;
                   for p = 1:nr
                       if(p == j)
                           continue
                       else
                           rows(z) = p;
                       end
                       z = z+1;
                   end
                % reduce
                AB(rows,:) = AB(rows,:) - AB(rows,k) .* AB(j,:);              
            end
            j = j+1;k = k+1;
        end
        
        F = AB(:,nr+1:end);
        
    end

end