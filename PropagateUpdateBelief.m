function[PropUpB,muB,sigB]=PropagateUpdateBelief(Bmu,Bsig,P,a,z)
% %predict:
muP = Bmu*P.F+a; 
sigP = P.F*Bsig*P.F'+P.sigW;
%update:
K = sigP*inv(sigP+P.sigV);
muB  =  muP+(z-muP)*K;
sigB = (eye(2)-K)*sigP; %sigP - K*sigP
PropUpB = mvnrnd(muB,sigB);
end