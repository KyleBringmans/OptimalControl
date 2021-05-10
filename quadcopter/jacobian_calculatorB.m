
syms u1 u2 u3 u4
syms x y z vx vy vz phi theta psi omegax omegay omegaz 
syms m L  k b g Kd Ixx Iyy Izz cm

inputs = [u1, u2, u3, u4]
formulas = [vx, vy,vz, -Kd*vx/m+k*cm/m*(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*(u1+u2+u3+u4),...
    -Kd*vy/m+k*cm/m*(cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi))*(u1+u2+u3+u4), ...
    -Kd*vz/m-g+k*cm/m*(cos(theta)*cos(phi))*(u1+u2+u3+u4), ...
    0,0,0, L*k*cm*(u1-u3)/Ixx-(Iyy-Izz)*omegay*omegaz/Ixx,...
    L*k*cm*(u2-u4)/Iyy-(Izz-Ixx)*omegax*omegaz/Iyy,...
    b*cm*(u1-u2+u3-u4)/Izz-(Ixx-Iyy)*omegax*omegay/Izz]

jacobian(formulas,inputs)

    
    
    
    