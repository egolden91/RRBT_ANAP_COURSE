function[LambdaNew]=PropagateError(Lambda,F)
%Predict:
LambdaNew   =  F*Lambda*F';
end