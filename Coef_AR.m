 function [coefficient,sigma2] = Coef_AR(Sig,ordre)
% Size_signal=length(Sig);
Toplitz=zeros(ordre+1,1);
autocorrelation_Sig=xcorr(Sig,'unbiased');
[maxi,position]=max(autocorrelation_Sig);
for i=1:ordre+1
    Toplitz(i,1)=autocorrelation_Sig(position+i-1);
end
Toeplitz_autocorr=toeplitz(Toplitz);
parametres=inv(Toeplitz_autocorr)*[1;zeros(ordre,1)];
coefficient=parametres/parametres(1);
sigma2=-1/parametres(1);
end

