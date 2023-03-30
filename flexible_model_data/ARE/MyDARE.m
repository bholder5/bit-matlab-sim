function P = MyDARE(A,B,Q,R)

% A has dimensions:  n*n
% Z has dimensions: 2n*2n

Z = [A + (B/R*B.')*(inv(A).')*Q  -(B/R*B.')*(inv(A).');...
    -inv(A).'*Q inv(A).'];

[V,D] = eig(Z);
eigvals = diag(D);
% Take only eigenvalues less than 0 as the basis
eigvalsmask = vecnorm(eigvals,2,2)<1;
n = size(A,1);
negV = zeros(2*n,n);

for i = 1:2*n
    if(eigvalsmask(i)>0)
        negV(:,i) = V(:,i);
    end
end

U11 = negV(1:n,:);        
U21 = negV(n+1:2*n,:);

P = real(U21/U11);
end