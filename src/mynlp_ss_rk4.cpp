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

    for(int i=0; i < (N_-1); i++){ 
        AD<double> x0 = sx_[i];
        AD<double> y0 = sy_[i];
        AD<double> tita0 = stita_[i];

        AD<double> V0 = x[V_start_ + i];
        AD<double> W0 = x[W_start_ + i];    

        AD<double> K1[3];
        AD<double> K2[3];
        AD<double> K3[3];
        AD<double> K4[3];

        K1[0] = dt_*V0*CppAD::cos(tita0);
        K1[1] = dt_*V0*CppAD::sin(tita0);
        K1[2] = dt_*W0;

        K2[0] = dt_*V0*CppAD::cos(tita0 + K1[2]/2);
        K2[1] = dt_*V0*CppAD::sin(tita0 + K1[2]/2);
        K2[2] = dt_*W0;

        K3[0] = dt_*V0*CppAD::cos(tita0 + K2[2]/2);
        K3[1] = dt_*V0*CppAD::sin(tita0 + K2[2]/2);
        K3[2] = dt_*W0;

        K4[0] = dt_*V0*CppAD::cos(tita0 + K3[2]);
        K4[1] = dt_*V0*CppAD::sin(tita0 + K3[2]);
        K4[2] = dt_*W0;

        sx_[i+1] = x0 + K1[0]/6 + K2[0]/3 + K3[0]/3 + K4[0]/6;
        sy_[i+1] = y0 + K1[1]/6 + K2[1]/3 + K3[1]/3 + K4[1]/6;
        stita_[i+1]= tita0 + K1[2]/6 + K2[2]/3 + K3[2]/3 + K4[2]/6;
                  
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
