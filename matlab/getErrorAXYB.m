function xyError = getErrorAXYB(X_f, Y_f, XActual, YActual)
% Compute the rotational and translational errors for X, Y and Z

xyError(1) = roterror(X_f, XActual);
xyError(2) = roterror(Y_f, YActual);

xyError(3) = tranerror(X_f, XActual);
xyError(4) = tranerror(Y_f, YActual);

end