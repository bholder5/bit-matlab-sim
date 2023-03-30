function P = MyCLYAP(A,Q)

    % Ap=q
    q = -MyVect(Q);
    n = size(A,1);

    A = kron(eye(n),A) + kron(conj(A),eye(n));
    p = A\q;
    P = MyMat(p,Q);

end

function v = MyVect(V)
    vectdims = size(V);
    v = zeros(vectdims(1)*vectdims(2),1);
    for i = 1:vectdims(2)
        v((1:vectdims(1)) + (i-1)*vectdims(1),:) = V(:,i);
    end
end

function M = MyMat(m,Q)
    Qsize = size(Q);
    M = zeros(Qsize(1),Qsize(2));
    for i = 1:Qsize(2)
        M(:,i) = m((1:Qsize(1)) + (i-1)*Qsize(1));
    end
end