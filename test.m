a = [0, 1, 2, 3];
b = [4, 5, 6, 7];
c = [a, b].';
d = c * 2;
e = zeros(3, 1);
b = [b 8]

testt(1) = 1;
testt(2) = 2;
testt(3) = 3;
size(testt.');
k_cl_m = 0.2
k_cl_rp = 0.2
k_cl_I = 0.2
k_cl_Izz = 0.3
k_cl_ri = 0.3
k_cl = [k_cl_m k_cl_rp*ones(1,3) k_cl_I*ones(1,5) k_cl_Izz k_cl_ri*ones(1,6)]
k_cl_gain = diag(k_cl)


zzzz = a_m(1, 2)

function x = a_m(a, b)
    function y = multi(a, b)
    y = a * b;
    end
    function z = add(a,b)
    z = a + b;
    end
x = multi(a, b) + add(a, b);
end

