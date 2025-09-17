function [out] = Eq2solve(m, m_p)
sigma_yield=250e6; %Yield stress of steel
rho_mat=8000;
rho_e=500; %The energy density of the material
h=30;
g=9.81;
%A little bit weird that the length of this structure canceled out for
%crushing.  Is that expected? 

out(1)= rho_mat*h*g/ sigma_yield*(m(1)+m_p+m(2))-m(2) ; 
out(2)= g*h*(m_p + m(1) + m(2)) - rho_e*m(1)*((m_p+m(1))/(m_p+m(1)+m(2) ));


end