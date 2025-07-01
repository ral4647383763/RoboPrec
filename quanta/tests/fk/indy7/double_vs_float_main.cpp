#include "double.hpp"
#include "float.hpp"
#include <random>
#include <iostream>

const int NUMBER_OF_TESTS = 100000;


int main(){

    double q[NU];
    double v[NU];
    double a[NU];
    double qcos[NU];
    double qsin[NU];  

    // run tests
    for(int i = 0; i < NUMBER_OF_TESTS; i++){
        // generate random qs
        for(int i = 0; i < NU; i++){
            q[i] = static_cast<double>(rand()) / RAND_MAX;
            v[i] = (static_cast<double>(rand()) / RAND_MAX) / 0.5;
            a[i] = static_cast<double>(rand()) / RAND_MAX;
        }

        // calculate qcos and qsin
        for(int i = 0; i < NU; i++){
            qcos[i] = cos(q[i]);
            qsin[i] = sin(q[i]);
        }


       auto res_double = FK_Indy7_double(
            qcos[0], qcos[1], qcos[2], qcos[3], qcos[4], qcos[5],
            qsin[0], qsin[1], qsin[2], qsin[3], qsin[4], qsin[5],
            v[0], v[1], v[2], v[3], v[4], v[5],
            a[0], a[1], a[2], a[3], a[4], a[5]
        );

        auto res_float = FK_Indy7_float(
            qcos[0], qcos[1], qcos[2], qcos[3], qcos[4], qcos[5],
            qsin[0], qsin[1], qsin[2], qsin[3], qsin[4], qsin[5],
            v[0], v[1], v[2], v[3], v[4], v[5],
            a[0], a[1], a[2], a[3], a[4], a[5]
        );
    
        for(int j = 0; j < NU; j++){
            for(int k = 0; k < 3; k++){
                for(int q = 0; q < 3; q++){
                    std::cout << res_double.oMis[j].rotation[k][q] - res_float.oMis[j].rotation[k][q] << " ";
                }
            }
        }
        std::cout << std::endl;
        for(int j = 0; j < NU; j++){
            for(int k = 0; k < 3; k++){
                std::cout << res_double.oMis[j].translation[k] - res_float.oMis[j].translation[k] << " ";
            }
        }
        std::cout << std::endl;
        for(int j = 0; j < NU; j++){
            for(int k = 0; k < 6; k++){
                std::cout << res_double.v[j][k] - res_float.v[j][k] << " ";
            }
        }
        std::cout << std::endl;
        for(int j = 0; j < NU; j++){
            for(int k = 0; k < 6; k++){
                std::cout << res_double.a[j][k] - res_float.a[j][k] << " ";
            }
        }
        std::cout << std::endl;

    }





}