function[LambdaNew]=PropagateUpdateError(Lambda,Sigma,K,F)
LambdaNew   =  F*Lambda*F';
LambdaNew   = LambdaNew +K*Sigma;
end
