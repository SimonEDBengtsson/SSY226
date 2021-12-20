function [u_body,u_world]=body_force_mpc(x,R,Qx,Qu,Qf,Ad,Bd,F_max)
    [u_world,x]=mpc(Ad,Bd,R,x,Qx,Qu,Qf,F_max);
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
function [ls,bs]=regular_polygonal_constraints(N,r,theta_0)
    ls=zeros(2,N);
    rs=zeros(2,N);
    bs=zeros(1,N);
    for i=1:N
        rs(1,i)=r*cos(theta_0+2*pi*(i-1)/N);
        rs(2,i)=r*sin(theta_0+2*pi*(i-1)/N);
    end
    for i=1:N-1
        M=[rs(1:2,i)' -1
           rs(1:2,i+1)' -1];
        L=null(M);
        ls(1:2,i)=L(1:2,1);
        bs(i)=L(3);
    end
    M=[rs(1:2,end)' -1
       rs(1:2,1)' -1];
    L=null(M);
    ls(1:2,end)=L(1:2,1);
    bs(end)=L(3);
    ls=ls.*[sign(bs);sign(bs)];
    bs=abs(bs);
end
function [A_lt,b_lt]=inequality_constraints(N,nEdges,F_max)
    [ls,bs]=regular_polygonal_constraints(nEdges,F_max,pi/nEdges);
    A_lt=zeros(nEdges*N,3*(N+1));
    iRows=1:nEdges;
    iCols=1:2;
    for i=1:N
        A_lt(iRows,iCols)=ls';
        iRows=iRows+nEdges;
        iCols=iCols+3;
    end
    b_lt=repmat(bs',N,1);
end
function [u,x]=mpc(A,B,R,x0,Qx,Qu,Qf,F_max)
    R=R(:);
    [nStates,nControls]=size(B);
    N=size(R,1)/nStates-1;
    free_response=state_matrix(A,N)*x0;
    M=control_matrix(A,B,N);
    Qx=blkdiag(cost_matrix(Qx,N-1),Qf);
    Qu=cost_matrix(Qu,N);
    H=M'*Qx*M+Qu;
    f=-M'*Qx*(R-free_response);
    [A_lt,b_lt]=inequality_constraints(N,8,F_max);
    options=optimoptions('quadprog','Algorithm','active-set','Display','off');
    U=quadprog(H,f,A_lt,b_lt,[],[],[],[],-H\f,options);
    X=free_response+M*U;
    u=reshape(U,nControls,[]);
    x=reshape(X,nStates,[]);
end