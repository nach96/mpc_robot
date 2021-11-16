#include "mynlp.hpp"
/**
 * @brief Single shooting approach.
 * 
 */
class FG_eval_ss : public FG_eval
{
    public:
    FG_eval_ss(int N, AD<double> dt, int div, double xp,  double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5, double preV, double preW) :
    FG_eval( N, dt, xp, yp, ap, dist, ang, yaw, K1, K2, K3, K4, K5, preV, preW){
        div_ = div;
    };
    void operator()(ADvector& fg, const ADvector& x);

    private:
    int div_;
    //Create internal variables to calculate state by integration.
    //AD<double> *sx_; //[N_];
    //AD<double> *sy_; //[N_];
    //AD<double> *stita_; //[N_];
    
};

class myNLP_ss : public myNLP
{
    public:
    myNLP_ss(double N, double dt, int div, double vmax, double wmax) : myNLP( N,  dt,  vmax,  wmax){
        div_ = div;
    };
    void my_solve(int N, double dt, double xp, double yp, double ap,double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5);
    private:
    int div_;
};