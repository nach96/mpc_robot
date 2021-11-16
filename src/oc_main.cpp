#include "mynlp_ss.hpp"
#include "mynlp_ms.hpp"
#include "mynlp_fd.hpp"
#include "old_mynlp.hpp"
#include "ros/ros.h"


int main(int argc, char *argv[]){
    ros::init(argc,argv,"main_node");
    ros::NodeHandle n;

    std::string method = "ss";
    int N = 40;
    double dt=0.1;
    double dist = 3.0; // Desired distance between robot and person
    double ang = 0.0; //Desired tita of the robot
    double yaw = 0.0; //Desired yaw of the camera (Perspective to look at the person)
    double K1 = 1.0; //Weight of distance
    double K2 = 1.0; //Weight of ang
    double K3 = 1.0; //Weigth of yaw
    double K4 = 0.5; //Weigth of yaw
    double K5 = 0.5; //Weigth of yaw    
    double vmax = 1.5;
    double wmax = 0.78;
    double xpr = 3.0;
    double ypr = 0.0;
    double titapr = 0.0;
    int div = 2;

    bool get_param = true;
    if (get_param == true){
        n.getParam("horizon_N",N);
        n.getParam("control_step",dt);
        n.getParam("distance", dist);
        n.getParam("angle_tita", ang);
        n.getParam("yaw", yaw);
        n.getParam("K_distance", K1);
        n.getParam("K_ang", K2);
        n.getParam("K_yaw", K3);
        n.getParam("K_v", K4);
        n.getParam("K_w", K5);
        n.getParam("v_max", vmax);
        n.getParam("w_max", wmax);
        n.getParam("method",method);
        n.getParam("xpr",xpr);
        n.getParam("ypr",ypr);
        n.getParam("titapr",titapr);
        n.getParam("subdivision_per_step",div);
        //n.getParam("useGroundTruth", useGroundTruth);
    }

    if(method=="ss"){
        std::cout << "---------------Single Shooting method----------------------- \n";
        myNLP_ss mynlp(N,dt,div,vmax,wmax);
        mynlp.my_solve(N,dt,xpr,ypr,titapr, dist, ang, yaw, K1, K2, K3, K4, K5);    //Solve nlp 
    }else if(method == "ms"){
        std::cout << "---------------Multiple Shooting method----------------------- \n";
        myNLP_ms mynlp(N,dt,div,vmax,wmax);
        mynlp.my_solve(N,dt,xpr,ypr,titapr, dist, ang, yaw, K1, K2, K3, K4, K5);    //Solve nlp
        //mynlp.my_solve(N,dt,xpr,ypr,titapr, dist, ang, yaw, K1, K2, K3, K4, K5);    //Solve nlp 
    }else if(method =="fd"){
        std::cout << "-----------------Full Discretization method ------------------------\n";
        myNLP_fd mynlp(N,dt,vmax,wmax);
        mynlp.my_solve(N,dt,xpr,ypr,titapr, dist, ang, yaw, K1, K2, K3, K4, K5);    //Solve nlp
        //mynlp.my_solve(N,dt,xpr,ypr,titapr, dist, ang, yaw, K1, K2, K3, K4, K5);    //Solve nlp 
    }else if(method=="old"){
        myNLP_old mynlp(vmax, wmax);
        mynlp.my_solve(xpr,ypr,titapr, dist, ang, yaw, K1, K2, K3, K4, K5);
    }

    ros::spinOnce();    
    return 0;
}
    