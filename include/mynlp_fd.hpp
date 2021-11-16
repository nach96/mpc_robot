#include "mynlp.hpp"
/**
 * @brief Full discretization approach.
 * 
 */
class FG_eval_fd : public FG_eval
{
    public:
    FG_eval_fd(int N, AD<double> dt, double xp,  double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5, double preV, double preW) :
    FG_eval( N, dt, xp, yp, ap, dist, ang, yaw, K1, K2, K3, K4, K5, preV, preW){
        x_start_ = W_start_ + N-1;
        y_start_ = x_start_ + N;
        tita_start_ = y_start_ + N;
    };
    void operator()(ADvector& fg, const ADvector& x); 
};

class myNLP_fd : public myNLP
{
    public:
    myNLP_fd(double N, double dt, double vmax, double wmax) : myNLP( N,  dt,  vmax,  wmax){};
    void my_solve(int N, double dt, double xp, double yp, double ap,double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5);
};