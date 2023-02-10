function[Estimate,muB,sigB,K]=PropagateUpdateEstimation(mu,sig,F,sigObsrvNoise,sigMoveNoise,z)
%predict:
muP = mu; 
sigP = F*sig*F'+sigMoveNoise;
%update:
K = sigP*inv(sigP+sigObsrvNoise);
muB  =  muP+(z-muP)*K;
sigB = (eye(2)-K)*sigP; %sigP - K*sigP
Estimate = mvnrnd(muB,sigB);
end