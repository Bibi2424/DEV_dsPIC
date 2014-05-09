SitesReference;
loadRobotParameters;

SitesPos = [startPos ; SitesPositions]
nbr = size(SitesPos,1);
for i=1:nbr
    for j=1:nbr
        dist(i,j) = sqrt((SitesPos(i,1)-SitesPos(j,1))^2+(SitesPos(i,2)-SitesPos(j,2))^2);
    end
end

dist