function [ L ] = Get_Lengths_E( Edges, x )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

N_L=size(Edges,1);
L=zeros(N_L,1);
for i=1:N_L
L(i)=norm(x(Edges(i,1),:)-x(Edges(i,2),:));
end

end

