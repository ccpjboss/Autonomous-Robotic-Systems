function [rhoNew,alphaNew,betaNew] = updateParameters(rho,alpha,beta,kRho,kAlpha,kBeta,T)
%UPDATEPARAMETERS Updates rho, beta and alpha parameters for movePose
dRho=-kRho*rho*cos(alpha);
dAlpha=kRho*sin(alpha)-kAlpha*alpha-kBeta*beta;
dBeta=-kRho*sin(alpha);

rhoNew = dRho*T+rho;
alphaNew = dAlpha*T+alpha;
alphaNew = atan2(sin(alphaNew),cos(alphaNew));
betaNew = dBeta*T+beta;
betaNew = atan2(sin(betaNew),cos(betaNew));
end

