function ret_img  = Image_undistortion(A)
%==== instrinsic
fx = 458.654;
fy = 457.296;
cx = 367.215;
cy = 248.375;
skew_c = 0;
%====

%==== distortion_coef
K1 = -0.28340811;
K2 = 0.07395907;
P2= 0.00019359;
P1 = 1.76187114e-05;

%=====

A = im2double(A);
sz = size(A);
height = sz(1);
width = sz(2);

%make canvas
a =Undistort(1,1,fx,fy,cx,cy,skew_c,K1,K2,P1,P2);
b = Undistort(width, height, fx,fy,cx,cy,skew_c,K1,K2,P1,P2);

h_len = b(2) - a(2);
w_len = b(1) - a(1);

for y=a(2)+1:b(2)
    for x =a(1)+1:b(1)
        u_p = DistortPixel(x,y,fx,fy,cx,cy,skew_c,K1,K2,P1,P2);
        px = u_p(1);
        py = u_p(2);
        if( px >=1 && py>=1 && px<width && py<height)
            x_vec(y-a(2), x-a(1)) = px;
            y_vec(y-a(2), x-a(1)) = py;
            px = int32(round(px));
            py = int32(round(py));
%             mat(y-a(2), x-a(1)) = A(py,px);
            
        else
            
        end
        
    end
end


 [x,y] = meshgrid(1:width, 1:height);
 rImg = interp2(x,y,A,x_vec,y_vec,'spline');


ret_img= rImg;
end

function ret_point = DistortPixel(px,py , fx ,fy ,cx ,cy ,skew_c , K1, K2, P1, P2)
x = px;
y = py;
%Normalize Point
p = Normalize(x,y,fx,fy,cx,cy,skew_c);
x= p(1);
y= p(2);
%distortion
p = Distort(x,y,K1,K2,P1,P2);
x= p(1);
y= p(2);
%Denormalize Point
p = Denormalize(x,y,fx,fy,cx,cy,skew_c);

ret_point = [p(1) p(2)];
end

function ret_point = Undistort(px ,py , fx,fy,cx,cy,skew_c,K1,K2,P1,P2)

xd = px;
yd = py;

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
rTmp = Denormalize(xu,yu,fx,fy,cx,cy,skew_c);

px = ceil(rTmp(1));
py = ceil(rTmp(2));

ret_point =[px py];
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