function ret_point =NormalizeAndUndistortPoint(px ,py , cameraParams,mode)
fx =cameraParams.FocalLength(1);
fy =cameraParams.FocalLength(2);
cx = cameraParams.PrincipalPoint(1);
cy = cameraParams.PrincipalPoint(2);
skew_c = cameraParams.Skew;
K1 =cameraParams.RadialDistortion(1);
K2 =cameraParams.RadialDistortion(2);
P1 =cameraParams.TangentialDistortion(1);
P2 = cameraParams.TangentialDistortion(2);
p_u = Normalize(px ,py,fx,fy,cx,cy,skew_c);

xd = p_u(1);
yd = p_u(2);
xu = xd;
yu = yd;

err_thr =(0.1/fx)*(0.1/fx) + (0.1/fy)*(0.1/fy);
while (1)
    xud = xu;
    yud = yu;
    tmp = Distort(xud, yud,K1,K2,P1,P2);
    xud = tmp(1);
    yud = tmp(2);
    err_x = xud-xd;
    err_y = yud-yd;
    err = err_x^2+err_y^2;
    xu = xu-err_x;
    yu = yu-err_y;
 
    if(err<err_thr)
        break;
    end
end
if(strcmp('norm',mode) || strcmp('Norm', mode))
    ret_point =[xu yu];
else
    rTmp = Denormalize(xu,yu,fx,fy,cx,cy,skew_c);
    
    px = ceil(rTmp(1));
    py = ceil(rTmp(2));
    
    ret_point =[px py];
end


end
function ret_point = Normalize(x ,y , fx,fy,cx,cy,skew_c)
y_n = (y-cy)/fy;
x_n = (x-cx)/fx - skew_c*y_n;
ret_point = [x_n y_n];
end

function ret_point = Distort(x,y,K1,K2,P1,P2)
radius2 = x^2 + y^2;
radial_d = 1+(K1 * (radius2)) + (K2 * (radius2^2));

x_d = radial_d*x + 2*P1*x*y + P2*(radius2 + 2 * (x^2));
y_d = radial_d*y + P1 * (radius2 + 2*(y^2)) + 2 * (P2 * x * y);

ret_point = [x_d y_d];
end
function ret_point = Denormalize(x,y ,fx,fy,cx,cy,skew_c)
x_p = fx*(x+skew_c*y) + cx;
y_p = fy*y +cy;

ret_point = [x_p y_p];
end