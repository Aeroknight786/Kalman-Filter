#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <bits/stdc++.h>
#include <fstream>
#include </home/aeroknight/eigen-3.3.7/Eigen/Eigen>
#include </home/aeroknight/eigen-3.3.7/Eigen/Dense>


using namespace cv;
using namespace std;
using namespace Eigen;

 //We will find the covariance matrices by finding standard deviation of first 15 values.
      double VarX = 25.13288,VarY = 36.25, VarVx = 20, VarVy = 20 ;
      double VarX1 = 5.13288,VarY1 = 6.25, VarVx1 = 0.04, VarVy1 = 0.2 ;
//Defining state variable [X,Y,Vx,Vy]
      double x,y,Vx,Vy,Vxo = 0,Vyo = 0;
      float Q = 0.05, W = 0;


Matrix<double,4,1> estimate ,meaState, Control;
double delta_t = 1;
Matrix4d I,A,Kalman,Cov,measCov;
Mat a(400,400,CV_8UC3,Scalar(0,0,0));



int main()
{
  A << 1, 0, delta_t, 0,
      0, 1, 0, delta_t,
      0, 0, 1, 0,
      0, 0, 0, 1;

Cov << VarX, 0, 0, 0,
      0, VarY, 0, 0,
      0, 0, VarVx, 0,
      0, 0, 0,VarVy ;
measCov <<VarX1, 0, 0, 0,
         0, VarY1, 0, 0,
        0, 0, VarVx1, 0,
         0, 0, 0,VarVy1 ;

I << 1, 0 ,0 , 0,
     0, 1, 0 , 0,
     0, 0, 1, 0,
     0 ,0 , 0, 1;

MatrixXd m = MatrixXd::Random(4,1);
 Control << 0.5 * (Vx - Vxo) * delta_t * delta_t ,
            0.5 * (Vy - Vyo) * delta_t * delta_t,
            (Vx - Vxo) * delta_t,
            (Vy - Vyo) * delta_t;
    cin >> x >> y;
    estimate << x,
            y,
           Vxo,
           Vyo;

namedWindow("Output",0);

for(int i=0;i<359;i++)
{
      cin >> x >> y >> Vx >> Vy;
      meaState << x,
                y,
               Vx,
               Vy;

      Cov = A  * Cov * A.transpose()+ (Q * MatrixXd::Random(4,4));
      estimate = A * estimate + Control + W * m;
      Kalman = Cov * ((Cov + measCov).inverse());
      Cov = (I - Kalman)*Cov;
      estimate = estimate + Kalman * (meaState - estimate);
     cout <<"State Variable{X,Y,Vx,Vy} = [" << estimate.transpose() << "\n\n\n Uncertainty: \n" <<Cov << "\n\n";
      Vxo = Vx; Vyo = Vy;
            //Storing past velocity to find acceleration of control variable.
            circle(a,Point(meaState(1,0),meaState(0,0)),1,Scalar(0,0,255),-1);
            circle(a,Point(estimate(1,0),estimate(0,0)),1,Scalar(0,255,0),-1);


}
imshow("Output",a);
waitKey(0);
return 0;
}
