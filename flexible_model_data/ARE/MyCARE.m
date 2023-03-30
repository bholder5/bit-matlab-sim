function P = MyCARE(A,B,Q,R)

% A has dimensions:  n*n
% Z has dimensions: 2n*2n

Z = [A -B/R*B.';-Q -A.'];

[V,D] = eig(Z);
eigvals = diag(D);
% Take only eigenvalues less than 0 as the basis
eigvalsmask = real(eigvals)<0;
n = size(A,1);
negV = zeros(2*n,n);
negV2 = negV;

cnt = 0;

for i = 1:2*n
    if(eigvalsmask(i)>0)
        cnt = cnt+1
        negV(:,cnt) = V(:,i);
    end
end

U11 = negV(1:n,:);        
U21 = negV(n+1:2*n,:);

P = (U21/U11);

negV2 = negV
negV2(:,2) = negV(:,3);
negV2(:,3) = negV(:,2);
negV2
negV
U11 = negV2(1:n,:);
U21 = negV2(n+1:2*n,:);
P2 = (U21/U11);

end