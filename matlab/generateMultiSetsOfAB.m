function  [A, B] = ...
    generateMultiSetsOfAB(n_sets, num, optPDF, gmean, cov, XActual, YActual)

A = zeros(4, 4, num, n_sets);
B = zeros(4, 4, num, n_sets);

for i = 1 : n_sets
    [ A(:,:,:,i), B(:,:,:,i) ] = generateSetsOfAB(num, optPDF, gmean, cov, ...
                                                    XActual, YActual);
end

end