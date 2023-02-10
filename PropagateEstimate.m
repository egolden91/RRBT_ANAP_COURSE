function[Estimate,muNew,sigNew]=PropagateEstimate(mu,sig,F,sigW)
%Predict:
muNew    = mu;
sigNew   =  F*sig*F'+sigW;
Estimate = mvnrnd(muNew,sigNew);
end