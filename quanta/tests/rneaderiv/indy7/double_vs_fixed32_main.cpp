#include "double.hpp"
#include "fixed32.hpp"
#include <random>
#include <iostream>

const int NUMBER_OF_TESTS = 1e5;


int main(){

    double q[NU];
    double v[NU];
    double a[NU];
    double qcos[NU];
    double qsin[NU];  

    std::random_device rd;
    std::mt19937 gen(rd());
    int min_val_q = -3.14;
    int max_val_q = 3.14;
    int min_val_v = -0.5;
    int max_val_v = 0.5;
    int min_val_a = -1;
    int max_val_a = 1;

    std::uniform_real_distribution<double> dis_q(min_val_q, max_val_q);
    std::uniform_real_distribution<double> dis_v(min_val_v, max_val_v);
    std::uniform_real_distribution<double> dis_a(min_val_a, max_val_a);

    // run tests
    for(int i = 0; i < NUMBER_OF_TESTS; i++){
        // generate random qs
        for(int i = 0; i < NU; i++){
            q[i] = dis_q(gen);
            v[i] = dis_v(gen);
            a[i] = dis_a(gen);
        }

        // calculate qcos and qsin
        for(int i = 0; i < NU; i++){
            qcos[i] = cos(q[i]);
            qsin[i] = sin(q[i]);
        }


        auto res_double = RNEADERIV_Indy7_double(
            qcos[0], qcos[1], qcos[2], qcos[3], qcos[4], qcos[5],
            qsin[0], qsin[1], qsin[2], qsin[3], qsin[4], qsin[5],
            v[0], v[1], v[2], v[3], v[4], v[5],
            a[0], a[1], a[2], a[3], a[4], a[5]
        );

        auto res_float = RNEADERIV_Indy7_fixed32(
            qcos[0], qcos[1], qcos[2], qcos[3], qcos[4], qcos[5],
            qsin[0], qsin[1], qsin[2], qsin[3], qsin[4], qsin[5],
            v[0], v[1], v[2], v[3], v[4], v[5],
            a[0], a[1], a[2], a[3], a[4], a[5]
        );
    
        // print all differences
        for(int j = 0; j < NU; j++){
            for(int k = 0; k < NU; k++){
                std::cout << res_double.partial_da[j][k] - res_float.partial_da[j][k] << " ";
            }
        }
        std::cout << std::endl;
        for(int j = 0; j < NU; j++){
            for(int k = 0; k < NU; k++){
                std::cout << res_double.partial_dv[j][k] - res_float.partial_dv[j][k] << " ";
            }
        }
        std::cout << std::endl;
        for(int j = 0; j < NU; j++){
            for(int k = 0; k < NU; k++){
                std::cout << res_double.partial_dq[j][k] - res_float.partial_dq[j][k] << " ";
            }
        }
        std::cout << std::endl;

    }





}