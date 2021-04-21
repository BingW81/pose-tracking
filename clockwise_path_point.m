function [x_p,y_p,curv_clockwise]=clockwise_path_point(path_point)
[~,ind]=min(path_point(:,1));
Predict_point=zeros(221,2);
Predict_point(1,:)=path_point(ind,:);
path_point(ind,:)=zeros(1,2);

for j=1:(length(path_point(:,1)))
    for i=1:length(path_point(:,1))    
        dis(i)=sqrt((Predict_point(j,1)-path_point(i,1))^2+(Predict_point(j,2)-path_point(i,2))^2);
    end
[~,ind1]=min(dis);
Predict_point(j+1,:)=path_point(ind1,:);
curv_clockwise(j,1)=(Predict_point(j+1,2)-Predict_point(j,2))/(Predict_point(j+1,1)-Predict_point(j,1));
path_point(ind1,:)=zeros(1,2);
end
x_p=Predict_point(1:length(path_point(:,1)),1);
y_p=Predict_point(1:length(path_point(:,1)),2);

end