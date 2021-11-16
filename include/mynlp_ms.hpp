#include "mynlp.hpp"
/**
 * @brief Multiple shooting approach. TODO.
 * 
 */
class FG_eval_ms : public FG_eval
{
    public:
    FG_eval_ms(int N, AD<double> dt, int div, double xp,  double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5, double preV, double preW) :
    FG_eval( N, dt, xp, yp, ap, dist, ang, yaw, K1, K2, K3, K4, K5, preV, preW){
        x_start_ = W_start_+N-1;;
        y_start_ = x_start_ + N;
            tita_start_ = y_start_ + N;
        div_ = div;
    };
    void operator()(ADvector& fg, const ADvector& x);

    private:
    int div_; 
};

class myNLP_ms : public myNLP
{
    public:
    myNLP_ms(double N, double dt, int div, double vmax, double wmax) : myNLP( N,  dt,  vmax,  wmax){
        div_=div;
    };
    void my_solve(int N, double dt, double xp, double yp, double ap,double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5);
    private:
    size_t x_start_;// = 0;
    size_t y_start_;// = x_start + N;
    size_t tita_start_; // = y_start + N;
    int div_;
};
  
