close all
vds = 0:0.1:4;
hold on
vt = 1;
k = 50;
for vgs = 0:5
    I = 0.*((vgs-vt) < 0 & 0  <= vds) + k*((vgs-vt)*vds-0.5*vds.^2).*(0<=vds & vds<=(vgs-vt))+...
        (0.5*k*(vgs-vt).^2).*(0<(vgs-vt) & (vgs-vt)<vds);
    plot(vds,I)
end

ids = (0.5*k*(vds).^2);
plot(vds,ids,'r')