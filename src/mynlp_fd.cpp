#include "mynlp_fd.hpp"

void FG_eval_fd::operator()(ADvector& fg, const ADvector& x)
{
    assert(fg.size() == 1 + 3*(N_-1) ); //f+ eq_constraints + accel ineq
    assert(x.size() == 3*N_ + 2*(N_-1) );

    //-------------------------------- f(x) Objective Function --------------------------------------------------------------
    fg[0]=0;
    
    for(int i=0; i < N_; i++){  
        //Calculate objective variables
        AD<double> d = CppAD::pow( CppAD::pow(x[x_start_+i]-xp_,2) + CppAD::pow(x[y_start_+i]-yp_,2), 0.5);
        AD<double> delta = (x[tita_start_+i]-ap_);
        AD<double> yaw = CppAD::atan2(x[y_start_+i]-yp_ , x[x_start_+i]-xp_);// - ap_;

        //Evaluate objective function
        fg[0] += K1_*CppAD::pow( d - dist_ , 2)*(i+1); //CppAD::pow((i+1),2);  
        fg[0] += K2_*CppAD::pow( delta - ang_, 2)*(i+1);// CppAD::pow((i+1),2);
        fg[0] += K3_*CppAD::pow( yaw - yaw_, 2)*(i+1);// CppAD::pow((i+1),2);
    }
    //Penalize actions    
    for(int i=0; i<N_-1; i++){
        fg[0] += K4_*CppAD::pow(x[V_start_+i],2)* (i+1);// CppAD::pow((i+1),2);
        fg[0] += K5_*CppAD::pow(x[W_start_+i],2)* (i+1);// CppAD::pow((i+1),2);
    }
    /*
    //Last state higher weight
    AD<double> d = CppAD::pow( CppAD::pow(x[x_start_+N_-1]-xp_,2) + CppAD::pow(x[y_start_+N_-1]-yp_,2), 0.5);
    AD<double> delta = (x[tita_start_+N_-1]-ap_);
    AD<double> yaw = CppAD::atan2(x[y_start_+N_-1]-yp_ , x[x_start_+N_-1]-xp_);// - ap_;
    fg[0] += K1_*CppAD::pow( d - dist_ , 2);  
    fg[0] += K2_*CppAD::pow( delta - ang_, 2);
    fg[0] += K3_*CppAD::pow( yaw - yaw_, 2);
    */

    // Constraints (Dynamic equation discretized)
    for(int i=1,j=1; i < N_; i++,j+=3){
        AD<double> x0 = x[x_start_ + i -1];
        AD<double> y0 = x[y_start_ + i -1];
        AD<double> tita0 = x[tita_start_ + i -1];
        AD<double> V0 = x[V_start_ + i -1];
        AD<double> W0 = x[W_start_ + i -1];

        AD<double> x1 = x[x_start_ + i];
        AD<double> y1 = x[y_start_ + i];
        AD<double> tita1 = x[tita_start_ + i];
        AD<double> V1 = x[V_start_ + i];
        AD<double> W1 = x[W_start_ + i];
        
        fg[j] = x1 - x0 - dt_*CppAD::cos(tita0)*V0;
        fg[j+1] = y1 - y0 - dt_*CppAD::sin(tita0)*V0;
        fg[j+2] = tita1 - tita0 - dt_*W0;
        
       /*
       //Implicit Euler
       fg[j] = x1 - x0 - dt_*CppAD::cos(tita1)*V1;
        fg[j+1] = y1 - y0 - dt_*CppAD::sin(tita1)*V1;
        fg[j+2] = tita1 - tita0 - dt_*W1;
       */
      /*
      //Function integrator
       AD<double> x_i1[3] = {x1,y1,tita1};
       AD<double> x_i[3] = {x0,y0,tita0};
       euler(x_i1, x_i, {V0,W0}, 0.1);
       */

    }
    return;
}


void myNLP_fd::my_solve(int N, double dt, double xp, double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5){
    V_start_ = 0;
    W_start_ = V_start_ + N-1; //-1 as last actions are not applied
    x_start_ = W_start_ + N-1;
    y_start_ = x_start_ + N;
    tita_start_ = y_start_ + N;
    // number of independent variables (domain dimension for f and g)
    size_t nx = 3*N + 2*(N-1);

    // number of constraints (range dimension for g)
    size_t ng =  3*(N-1);

    Dvector x_i(nx);

    for(int i=0; i<nx; i++){
        if(nx<2*(N-1)){
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
    
    // 0 Equality constraints. (Dynamic equations)
    for (int i=0; i<ng; i++){
        g_l[i] = g_u[i] = 0.0;
    }

    // Object to evaluate objective and constraints
    FG_eval_fd fg_eval(N,dt,xp,yp,ap, dist, ang, yaw, K1, K2, K3, K4, K5, preV_, preW_);

    std::string options = set_options();
    
    assert(x_l.size()==nx);
    assert(g_l.size()==ng);
    
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval_fd>(options, x_i, x_l, x_u, g_l, g_u, fg_eval, solution_);

    //Save the soultion
    save_solution(solution_,N);
}
