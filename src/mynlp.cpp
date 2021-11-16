#include "mynlp.hpp"
#include <vector>


//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////          FG_eval          //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

FG_eval::FG_eval(int N, AD<double> dt, double xp,  double yp, double ap, double dist, double ang, double yaw, double K1, double K2, double K3, double K4, double K5, double preV, double preW){
  //Variable sizes
  dt_=dt;
  N_=N;
  V_start_ = 0;
  W_start_ = V_start_ + N_-1; //-1 as last actions are not applied
  //Person position
  xp_ = xp;
  yp_ = yp;
  ap_ = ap;
  //Objective variables
  dist_ = dist;
  ang_ = ang;
  yaw_ = yaw;
  //Parameters to tune behaviour
  K1_ = K1;
  K2_ = K2;
  K3_ = K3;
  K4_ = K4;
  K5_ = K5;
  preV_ = preV;
  preW_ = preW;
}

//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////          myNLP            //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

myNLP::myNLP(double N, double dt, double vmax, double wmax){
  //N_=N;
  //dt_=dt;
  vmax_ = vmax;
  wmax_ = wmax;
  //V_start_ = 0;
  //W_start_ = V_start_ + N-1; //-1 as last actions are not applied
}  

std::string myNLP::set_options(void){
   std::string options;
    // turn off any printing
    options += "Integer print_level  5\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     200\n";
    options += "Numeric tol          1e-8\n";
    //options += "String  derivative_test            second-order\n";
    //options += "Numeric derivative_test_tol          0.01\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    //File urs/include/cppad/ipopt/solve_callback.hpp modified to warm-start also if( solution.x.size()>0 and init_z == true).
    //If this options are set, warm-start will automatically begin from second step.
    //But apparently it doesn't improves as much as it should
    if(solution_.x.size()>0){
      std::cout << "Yep, going warm-start. \n";
      options += "String nlp_scaling_method none\n"; 
      options += "String warm_start_init_point yes\n";

      options += "Numeric warm_start_bound_frac 1e-16\n";
      options += "Numeric warm_start_bound_frac 1e-16\n";
      options += "Numeric warm_start_bound_push 1e-16\n";
      options += "Numeric warm_start_mult_bound_push 1e-16\n";
      options += "Numeric warm_start_slack_bound_frac 1e-16\n";
      options += "Numeric warm_start_slack_bound_push 1e-16\n";     
    }
    return options;
}

void myNLP::save_solution(CppAD::ipopt::solve_result<Dvector>solution, int N){
    std::vector<double> Vr(&solution.x[V_start_], &solution.x[W_start_-1]);
    std::vector<double> Wr(&solution.x[W_start_], &solution.x[W_start_+N-2]);

    //printf("Vr:");
    std::cout << " \n";
    std::cout << "Vr = [";
    for(auto it = Vr.begin() ; it != Vr.end(); it++){
      std::cout << *it << " ";
    }
    //printf("Wr:");
    std::cout << "]; \n";
    std::cout << "Wr = [";  
    for(auto it = Wr.begin() ; it != Wr.end(); it++){
      std::cout << *it << " ";
    }
    std::cout << "]; \n";

    return;
}