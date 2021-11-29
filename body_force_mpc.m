function [u_body,u_world]=body_force_mpc(x,R,Qx,Qu,Qf,Ad,Bd)
    [u_world,x]=mpc(Ad,Bd,R,x,Qx,Qu,Qf);
    psis=x(3,:);
    u_body=u_world;
    for i=1:length(psis)
        psi=psis(i);
        R=[cos(psi) sin(psi)
          -sin(psi) cos(psi)];
        u_body(1:2,i)=R*u_world(1:2,i);
    end
end

function M=state_matrix(A,N)
    nStates=length(A);
    M=zeros(nStates*(N+1),nStates);
    iRows=1:nStates;
    M(iRows,:)=eye(nStates);
    iRows=iRows+nStates;
    for iTau=1:N
        M(iRows,:)=A*M(iRows-nStates,:);
        iRows=iRows+nStates;
    end
end
function M=control_matrix(A,B,N)
    [nStates,nControls]=size(B);
    M=zeros(nStates*(N+1),nControls*(N+1));
    iRows=(1:nStates)+nStates;
    iCols=1:nControls;
    for iState=1:N
        M(iRows,:)=A*M(iRows-nStates,:);
        M(iRows,iCols)=B;
        iRows=iRows+nStates;
        iCols=iCols+nControls;
    end
end
function Q_diag=cost_matrix(Q,N)
    nValues=length(Q);
    Q_diag=zeros(nValues*(N+1));
    is=1:nValues;
    for iValue=0:N
        Q_diag(is,is)=Q;
        is=is+nValues;
    end
end
function [u,x]=mpc(A,B,R,x0,Qx,Qu,Qf)
    R=R(:);
    [nStates,nControls]=size(B);
    N=size(R,1)/nStates-1;
    free_response=state_matrix(A,N)*x0;
    M=control_matrix(A,B,N);
    Qx=blkdiag(cost_matrix(Qx,N-1),Qf);
    Qu=cost_matrix(Qu,N);
    U=(M'*Qx*M+Qu)\M'*Qx*(R-free_response);
    X=free_response+M*U;
    u=reshape(U,nControls,[]);
    x=reshape(X,nStates,[]);
end