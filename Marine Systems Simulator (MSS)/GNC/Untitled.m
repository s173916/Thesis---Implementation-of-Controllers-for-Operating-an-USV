

u = 0:0.1:5

x_u = 53.4
X_uu = 0.45*72.3


T = ((x_u + x_uu .* abs(u)) .* abs(u))/2/2

plot(u,T)