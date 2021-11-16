#include "mynlp_ss.hpp"

void FG_eval_ss::operator()(ADvector& fg, const ADvector& x)
{
    assert(fg.size() == 1);
    assert(x.size() == 2*(N_-1) ); //  State is only actions

    //-------------------------------- f(x) Objective Function --------------------------------------------------------------
    fg[0]=0;
    AD<double> sx_[div_*N_];
    AD<double> sy_[div_*N_];
    AD<double> stita_[div_*N_];
    sx_[0]=0;
    sy_[0]=0;
    stita_[0]=0;

    int i = 0;
    for(int j=0; j < (N_-1); j++){ 
        for(int k=0; k<div_; k++){
            //Integrate state: Euler
            sx_[i+1] = sx_[i] + dt_/div_*CppAD::cos(stita_[i])*x[V_start_+j];
            sy_[i+1] = sy_[i] + dt_/div_*CppAD::sin(stita_[i])*x[V_start_+j];
            stita_[i+1] = stita_[i] + dt_/div_*x[W_start_+j];
            i++;
        }                   
        //Calculate objective variables
        AD<double> d = CppAD::pow( CppAD::pow(sx_[i] -xp_,2) + CppAD::pow(sy_[i]-yp_,2), 0.5);
        AD<double> delta = (stita_[i]-ap_);
        AD<double> yaw = CppAD::atan2(sy_[i]-yp_, sx_[i] -xp_);
        //Evaluate objective function
        fg[0] += K1_*CppAD::pow( d - dist_ , 2)*(i+1);  
        fg[0] += K2_*CppAD::pow( delta - ang_,2)*(i+1);
        fg[0] += K3_*CppAD::pow( yaw - yaw_,2)*(i+1);
    }
      //Penalize actions    
    for(int i=0; i<N_-1; i++){
        fg[0] += K4_*CppAD::pow(x[V_start_+i],2)*(i+1);
        fg[0] += K5_*CppAD::pow(x[W_start_+i],2)*(i+1);
    }
}


void myNLP_ss::my_solve(int N, double dt, double xp, double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5){
    V_start_ = 0;
    W_start_ = V_start_ + N-1; //-1 as last actions are not applied

    // number of independent variables (domain dimension for f and g)
    size_t nx = 2*(N-1);

    // number of constraints (range dimension for g)
    size_t ng1 = 0;
    size_t ng =  0;

    Dvector x_i(nx);

    for(int i=0; i<nx; i++){
        x_i[i]=0.1;
    }  
    //Lower and upper limits for x
    Dvector x_l(nx), x_u(nx);
    // lower and upper limits for g
    Dvector g_l(ng), g_u(ng);

    //Action limits
    for(int i=0; i<N-1; i++){
        x_l[V_start_ + i] = -vmax_;//V_r
        x_u[V_start_ + i] = +vmax_;

        x_l[W_start_ + i] = -wmax_;//w_r
        x_u[W_start_ + i] = +wmax_;
    }
    
    // 0 Equality constraints. (Dynamic equations)

    // Object to evaluate objective and constraints
    FG_eval_ss fg_eval(N,dt,div_,xp,yp,ap, dist, ang, yaw, K1, K2, K3, K4, K5, preV_, preW_);

    std::string options = set_options();
    
    assert(x_l.size()==nx);
    assert(g_l.size()==ng);
    
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval_ss>(options, x_i, x_l, x_u, g_l, g_u, fg_eval, solution_);

    //Save the soultion
    save_solution(solution_,N);
}
