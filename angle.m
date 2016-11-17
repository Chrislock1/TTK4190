function phi = angle(v1,v2)

v_dot = v1*v2';                                                 % Dot product
v_abs = (v1(1).^2+v1(2).^2).^0.5*(v2(1).^2+v2(2).^2).^0.5;      % Absolute product

phi = acos(v_dot/v_abs);

