function [ X ] = LG_output_free( xfree)
% free mode: generate all states based on 18 states.
% 
Lc = 0.5;
%% There is no problem in the posQ, posP
xQ = xfree(1);dxQ = xfree(2);
yQ = xfree(3);dyQ = xfree(4);
zQ = xfree(5);dzQ = xfree(6);

xP = xfree(13);dxP = xfree(14);
yP = xfree(15);dyP = xfree(16);
zP = xfree(17);dzP = xfree(18);
%% the only problem is how to define/get alpha and beta in free mode:
posQ = [xQ;yQ;zQ];
posP = [xP;yP;zP];
dis = norm(posQ-posP,2); % distance between Q and P
if dis < Lc % if distance < cable length, alpha and beta are defined as 0.
    beta = 0.0;
    alpha = 0.0;
    dbeta = 0.0;
    dalpha = 0.0;
else % if distance == cable
    beta = asin((xP-xQ)/Lc);
    alpha = asin((yP-yQ)/Lc/cos(beta));
    dbeta = (dxP-dxQ)/(Lc*cos(beta));
    dalpha = (dyP-dyQ+Lc*sin(beta)*sin(alpha)*dbeta)/(Lc*cos(beta)*cos(alpha));
end
   %x =    [PosQ             alpah,beta;            AttQ;        PosQ]
    X = [xfree(1:6);[alpha;dalpha;beta;dbeta;];xfree(7:18)];
end

