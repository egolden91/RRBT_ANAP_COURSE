function[ObsrvFlag] = CheckForObservation(ObsrvArea,X)
ObsrvFlag = 0;
[len,~] = size(ObsrvArea);
for ii = 1:len
    if X(1)>= ObsrvArea(ii,1) && X(1) <= ObsrvArea(ii,1)+ObsrvArea(ii,3) && X(2)<= ObsrvArea(ii,2)+ObsrvArea(ii,4) && X(1) >= ObsrvArea(ii,2)
        ObsrvFlag = 1;
        return
    end
end
end