#include "double.hpp"
#include <iostream>


int main(){
    double qcos_0_0, qcos_1_0, qcos_2_0, qcos_3_0, qcos_4_0, qcos_5_0;
    double qsin_0_0, qsin_1_0, qsin_2_0, qsin_3_0, qsin_4_0, qsin_5_0;
    double v_0_0, v_1_0, v_2_0, v_3_0, v_4_0, v_5_0;
    double a_0_0, a_1_0, a_2_0, a_3_0, a_4_0, a_5_0;

    std::cin >> qcos_0_0 >> qcos_1_0 >> qcos_2_0 >> qcos_3_0 >> qcos_4_0 >> qcos_5_0;
    std::cin >> qsin_0_0 >> qsin_1_0 >> qsin_2_0 >> qsin_3_0 >> qsin_4_0 >> qsin_5_0;
    std::cin >> v_0_0 >> v_1_0 >> v_2_0 >> v_3_0 >> v_4_0 >> v_5_0;
    std::cin >> a_0_0 >> a_1_0 >> a_2_0 >> a_3_0 >> a_4_0 >> a_5_0;

    auto res = RNEADERIV_Roarm3_double(
        qcos_0_0, qcos_1_0, qcos_2_0, qcos_3_0, qcos_4_0,
        qsin_0_0, qsin_1_0, qsin_2_0, qsin_3_0, qsin_4_0,
        v_0_0, v_1_0, v_2_0, v_3_0, v_4_0,
        a_0_0, a_1_0, a_2_0, a_3_0, a_4_0
    );

    for(int i = 0; i < NU; i++){
        for(int j = 0; j < NU; j++){
            std::cout << "partial_da[" << i << "][" << j << "]: " << res.partial_da[i][j] << "\n";
        }
    }
    for(int i = 0; i < NU; i++){
        for(int j = 0; j < NU; j++){
            std::cout << "partial_dv[" << i << "][" << j << "]: " << res.partial_dv[i][j] << "\n";
        }
    }
    for(int i = 0; i < NU; i++){
        for(int j = 0; j < NU; j++){
            std::cout << "partial_dq[" << i << "][" << j << "]: " << res.partial_dq[i][j] << "\n";
        }
    }
    
    return 0;
}