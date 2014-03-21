 
results = csvread('../results/step05.csv');
siz = nthroot(size(results,1),3);
X = reshape(results(:,2),siz,siz,siz);
Y = reshape(results(:,3),siz,siz,siz);
Z = reshape(results(:,1),siz,siz,siz);
V = reshape(results(:,4),siz,siz,siz);

figure;
hold on;
for a = 10.^(1:4) % 'a' defines the isosurface limits
    p = patch(isosurface(X,Y,Z,V,max(max(max(V)))/a)); % isosurfaces at max(V)/a
    isonormals(X,Y,Z,V,p); % plot the surfaces
    set(p,'FaceColor','red','EdgeColor','none'); % set colors
end
alpha(.1); % set the transparency for the isosurfaces
daspect([1 1 1]); box on; axis tight;



%test
[XX,YY,ZZ] = meshgrid(1:13,1:13,1:13)
VV = XX.*YY.*ZZ

figure;
hold on;
for a = 10.^(1:4) % 'a' defines the isosurface limits
    p = patch(isosurface(XX,YY,ZZ,VV,max(max(max(VV)))/a)); % isosurfaces at max(V)/a
    isonormals(XX,YY,ZZ,VV,p); % plot the surfaces
    set(p,'FaceColor','red','EdgeColor','none'); % set colors
end
alpha(.1); % set the transparency for the isosurfaces
daspect([1 1 1]); box on; axis tight;