#include "double.hpp"
#include "float.hpp"
#include <random>
#include <iostream>

const int NUMBER_OF_TESTS = 1e5;


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


        auto res_double = RNEA_Indy7_double(
            qcos[0], qcos[1], qcos[2], qcos[3], qcos[4], qcos[5],
            qsin[0], qsin[1], qsin[2], qsin[3], qsin[4], qsin[5],
            v[0], v[1], v[2], v[3], v[4], v[5],
            a[0], a[1], a[2], a[3], a[4], a[5]
        );

        auto res_float = RNEA_Indy7_float(
            static_cast<float>(qcos[0]), static_cast<float>(qcos[1]), static_cast<float>(qcos[2]), static_cast<float>(qcos[3]), static_cast<float>(qcos[4]), static_cast<float>(qcos[5]),
            static_cast<float>(qsin[0]), static_cast<float>(qsin[1]), static_cast<float>(qsin[2]), static_cast<float>(qsin[3]), static_cast<float>(qsin[4]), static_cast<float>(qsin[5]),
            static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]), static_cast<float>(v[3]), static_cast<float>(v[4]), static_cast<float>(v[5]),
            static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2]), static_cast<float>(a[3]), static_cast<float>(a[4]), static_cast<float>(a[5])
        );
    
        // print all differences
        for(int j = 0; j < NU; j++){
            std::cout << std::abs(res_double.taus[j] - res_float.taus[j]) << " ";
        }
        std::cout << std::endl;

    }





}