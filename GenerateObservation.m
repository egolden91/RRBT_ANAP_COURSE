function[z] = GenerateObservation(mu,sig)
%z = X+V
%mu is the nominal X
%Sigma is observation noise 
z = mvnrnd(mu,sig);
end