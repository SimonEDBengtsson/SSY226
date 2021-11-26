T=0.1; N=10;
Ac=[zeros(3) eye(3)
    zeros(3) zeros(3)];
Bc=[zeros(3)
    diag(1./[mass mass I(end)])];
sysc=ss(Ac,Bc,[],[]);
sysd=c2d(sysc,T);
Ad=sysd.A; Bd=sysd.B;
Qx=1e4*eye(6); Qu=1e-4*eye(3);
tau=0:T:100;
R=[cos(tau)
   sin(tau)
   tau+pi/2
   -sin(tau)
   cos(tau)
   1+0*tau];
x=[1;0;pi/2;0;0;0];
xs=zeros(length(x),length(tau)+1);
xs(:,1)=x;
for i=1:length(tau)
    R_horizon=R(:,i:min(i+N-1,end));
    u=mpc(Ad,Bd,R_horizon,x,Qx,Qu);
    x=Ad*x+Bd*u(:,1);
    xs(:,i+1)=x;
end
%%
hold on
plot(xs(1,:),xs(2,:),'.');
plot(R(1,:),R(2,:),'.');
axis equal

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
function [u,x]=mpc(A,B,R,x0,Qx,Qu)
    R=R(:);
    [nStates,nControls]=size(B);
    N=size(R,1)/nStates-1;
    free_response=state_matrix(A,N)*x0;
    M=control_matrix(A,B,N);
    Qx=cost_matrix(Qx,N);
    Qu=cost_matrix(Qu,N);
    U=(M'*Qx*M+Qu)\M'*Qx*(R-free_response);
    X=free_response+M*U;
    u=reshape(U,nControls,[]);
    x=reshape(X,nStates,[]);
end