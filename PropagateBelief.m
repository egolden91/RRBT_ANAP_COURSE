function[PropB,muP,sigP]=PropagateBelief(Bmu,Bsig,P,a)
%Predict:
muP = Bmu*P.F+a;
sigP = P.F*Bsig*P.F'+P.sigW;
PropB = mvnrnd(muP,sigP);
end