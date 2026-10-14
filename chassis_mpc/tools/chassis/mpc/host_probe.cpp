#include "chassis_mpc.h"
#include "chassis_tiny_data.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <limits>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

int main(int argc,char **argv) {
    chassis_mpc_result_t r;
    chassis_control_state_t x={}, ref={};
    if (chassis_mpc_solve(&x,&ref,&r)!=CHASSIS_MPC_NOT_INITIALIZED) return 2;
    chassis_mpc_init();
    const bool constrained=!(argc>1 && std::string(argv[1])=="unconstrained");
    const int iterations=argc>2 ? std::stoi(argv[2]):200;
    const double tol=argc>3 ? std::stod(argv[3]):0.001;
    TinySolver *s=whu_tiny_solver();
    s->settings->en_input_bound=constrained;
    s->settings->max_iter=iterations;
    s->settings->abs_pri_tol=tol;
    s->settings->abs_dua_tol=tol;
    x.x[0]=std::numeric_limits<float>::quiet_NaN();
    if(chassis_mpc_solve(&x,&ref,&r)!=CHASSIS_MPC_BAD_INPUT || r.valid) return 3;
    x.x[0]=0;
    std::cout<<std::setprecision(17);
    while(std::cin>>x.x[0]) {
        for(int i=1;i<10;++i) if(!(std::cin>>x.x[i])) return 4;
        for(int i=0;i<10;++i) if(!(std::cin>>ref.x[i])) return 4;
#ifdef _WIN32
        LARGE_INTEGER frequency,tick0,tick1;
        QueryPerformanceFrequency(&frequency); QueryPerformanceCounter(&tick0);
#else
        auto t0=std::chrono::steady_clock::now();
#endif
        int status=chassis_mpc_solve(&x,&ref,&r);
        if (status!=CHASSIS_MPC_OK) {
            if(r.valid) return 5;
            for(int i=0;i<4;++i) if(r.command.u[i]!=0) return 6;
        }
#ifdef _WIN32
        QueryPerformanceCounter(&tick1);
        double elapsed=1e6*(tick1.QuadPart-tick0.QuadPart)/frequency.QuadPart;
#else
        auto t1=std::chrono::steady_clock::now();
        double elapsed=std::chrono::duration<double,std::micro>(t1-t0).count();
#endif
        std::cout<<status<<' '<<r.valid<<' '<<r.iterations<<' '
                 <<elapsed<<' '
                 <<r.primal_residual<<' '<<r.dual_residual<<' '
                 <<r.input_bound_violation<<' '<<r.matrix_payload_bytes;
        for(int k=0;k<s->work->N-1;++k) for(int i=0;i<4;++i) std::cout<<' '<<s->solution->u(i,k);
        std::cout<<'\n';
    }
}
