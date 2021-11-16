#include "mynlp_ms.hpp"

void FG_eval_ms::operator()(ADvector& fg, const ADvector& x)
{
    assert(fg.size() == 1 + 3*(N_-1));
    assert(x.size() == 3*N_ + 2*(N_-1) ); //  State is actions and states on intervals

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
        AD<double> d = CppAD::pow( CppAD::pow(x[x_start_ +j] -xp_,2) + CppAD::pow(x[y_start_ + j]-yp_,2), 0.5);
        AD<double> delta = (x[tita_start_+j]-ap_);
        AD<double> yaw = CppAD::atan2(x[y_start_ +j]-yp_, x[x_start_ +j] -xp_);
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

    // Constraints Join shooting intervals
    for(int i=1,j=1,k=div_; i < N_; i++,j+=3, k+=div_){
        fg[j] =  x[x_start_+i] - sx_[k];
        fg[j+1] = x[y_start_+i] - sy_[k];
        fg[j+2] = x[tita_start_+i] - stita_[k];
    }
}


void myNLP_ms::my_solve(int N, double dt, double xp, double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5){
    V_start_ = 0;
    W_start_ = V_start_ + N-1; //-1 as last actions are not applied
    x_start_ = W_start_ + N-1;
    y_start_ = x_start_ + N;
    tita_start_ = y_start_ + N;
    // number of independent variables (domain dimension for f and g)
    size_t nx = 3*N + 2*(N-1);

    // number of constraints (range dimension for g)
    size_t ng =  3*(N-1); //N-1 Because state 0 is defined here and not as a constraint

    Dvector x_i(nx);

    for(int i=0; i<nx; i++){
        if(i<2*(N-1)){
            x_i[i]=0.1;
        }else{
            x_i[i]=0.0;
        } 
    }  
    //Lower and upper limits for x
    Dvector x_l(nx), x_u(nx);
    // lower and upper limits for g
    Dvector g_l(ng), g_u(ng);

    //Initial pose of the robot is always 0 because it is set as local reference.
    x_l[x_start_]=x_u[x_start_]=0.0; //Xr0
    x_l[y_start_]=x_u[y_start_]=0.0; //Yr0
    x_l[tita_start_]=x_u[tita_start_]=0.0; //Titar0

    //Space variable limits
    for(int i= 1; i<N; i++){
        x_l[x_start_ + i] = -1.0e19;//Xr
        x_u[x_start_ + i] = +1.0e19;

        x_l[y_start_ + i] = -1.0e19;//Yr
        x_u[y_start_ + i] = +1.0e19;

        x_l[tita_start_ + i] = -3.14;//tita_r
        x_u[tita_start_ + i] = +3.15;
    }
    //Action limits
    for(int i=0; i<N-1; i++){
        x_l[V_start_ + i] = -vmax_;//V_r
        x_u[V_start_ + i] = +vmax_;

        x_l[W_start_ + i] = -wmax_;//w_r
        x_u[W_start_ + i] = +wmax_;
    }
    
    // 0 Equality constraints. (Restriction continuity)
        for (int i=0; i<ng; i++){
        g_l[i] = g_u[i] = 0.0;
    }
    // Object to evaluate objective and constraints
    FG_eval_ms fg_eval(N,dt,div_,xp,yp,ap, dist, ang, yaw, K1, K2, K3, K4, K5, preV_, preW_);

    std::string options = set_options();
    
    assert(x_l.size()==nx);
    assert(g_l.size()==ng);
    
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval_ms>(options, x_i, x_l, x_u, g_l, g_u, fg_eval, solution_);

    //Save the soultion
    save_solution(solution_,N);
}
