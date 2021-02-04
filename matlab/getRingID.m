function ringID = getRingID(x,y,z)
  degree = atan(z/sqrt(x^2+y^2)) * 180 / pi;
  b0 = 3.250045704e+01;
  b1 = 6.925774161e-01;
  b2 = -1.533624820e-06;
  b3 = -3.205573315e-05;
  ringID = round(b0+b1*degree+b2*degree^2+b3*degree^3);
end