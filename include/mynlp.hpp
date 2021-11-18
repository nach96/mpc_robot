#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;
typedef CPPAD_TESTVECTOR( double ) Dvector;

//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////          FG_eval          //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
#ifndef FG_EVAL_H
#define FG_EVAL_H
/**
 * Class defining objective function (F) and constraints (G) to evaluate.
*/
class FG_eval 
{
public:
    typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
    FG_eval(int N, AD<double> dt, double xp,  double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5, double preV, double preW);
    ~FG_eval(){};
    virtual void operator()(ADvector& fg, const ADvector& x){};

protected:
  // Variables sizes
  int N_; //Horizon
  AD<double> dt_; //Control time step
  size_t V_start_; // = 0;
  size_t W_start_; // = V_start + N-1; //-1 as last actions are not applied
  size_t x_start_;// = W_start_+N-1;
  size_t y_start_;// = x_start + N;
  size_t tita_start_; // = y_start + N;
  // Person position
  double xp_;
  double yp_;
  double ap_;
  // Objective variables 
  AD<double> dist_;// = 3.0;  //[m]
  AD<double> ang_;// = 0.0; //[rad]
  AD<double> yaw_;// = -1.57; //[rad]
  // Parameters
  AD<double> K1_;// = 1.0; //Distance error
  AD<double> K2_;// = 1.0; //Tita error
  AD<double> K3_;// = 1.0; // Yaw error
  AD<double> K4_; // = 0.5; // acceleration limit
  AD<double> K5_; // = 0.5; //Angular acceleration limit
  AD<double> preV_;
  AD<double> preW_;
};
#endif //FG_EVAL_H


//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////          myNLP            //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
#ifndef MYNLP_H
#define MYNLP_H
/**
 * Main class defining de Non Linear Program 
*/
class myNLP
{
public:
    myNLP(double N, double dt, double vmax, double wmax);
    virtual void my_solve(int N, double dt, double xp, double yp, double ap,double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5){};
    void save_solution(CppAD::ipopt::solve_result<Dvector> solution, int N);
    std::string set_options(void);
    std::vector<double> getV(){return Vr_;};
    std::vector<double> getW(){return Wr_;};

    //void set_limits(Dvector x_l, Dvector x_u, Dvector g_l, Dvector g_u);

    /*
    void ode_model(AD<double> dx[], AD<double> x[], AD<double> actions[]){
        dx[0] = actions[0]*CppAD::cos(x[2]);
        dx[1] = actions[0]*CppAD::sin(x[2]);
        dx[2] = actions[1];
    };
    void euler(AD<double> x1[],AD<double> x0[], AD<double> actions[], double dt){
        AD<double> dx[3];
        ode_model(dx, x0,actions);
        for(int i = 0; i<3; i++){
            x1[i] = x0[i] + dt*dx[i];
        }
    };
    */
protected: 
    size_t V_start_; // = 0;
    size_t W_start_; // = V_start + N-1; //-1 as last actions are not applied
    size_t x_start_;// = W_start + N-1;
    size_t y_start_;// = x_start + N;
    size_t tita_start_; // = y_start + N;

    double vmax_; // = 1.5;//E3; //[mm/s]
    double wmax_; // = 0.78; //[rad/s]

    double preV_;
    double preW_;
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution_;
    double av_ = 1*0.1; //Maximum long acceleration
    double aw_ = 1*0.1;
    //Calculated actions
    std::vector<double> Vr_;
    std::vector<double> Wr_;
};
#endif // MYNLP_H
