#include "fixed32_no_conversion.hpp"
#include <iostream>


int main(){
    int32_t qcos_0_0, qcos_1_0, qcos_2_0, qcos_3_0, qcos_4_0, qcos_5_0;
    int32_t qsin_0_0, qsin_1_0, qsin_2_0, qsin_3_0, qsin_4_0, qsin_5_0;
    int32_t v_0_0, v_1_0, v_2_0, v_3_0, v_4_0, v_5_0;
    int32_t a_0_0, a_1_0, a_2_0, a_3_0, a_4_0, a_5_0;

    std::cin >> qcos_0_0 >> qcos_1_0 >> qcos_2_0 >> qcos_3_0 >> qcos_4_0 >> qcos_5_0;
    std::cin >> qsin_0_0 >> qsin_1_0 >> qsin_2_0 >> qsin_3_0 >> qsin_4_0 >> qsin_5_0;
    std::cin >> v_0_0 >> v_1_0 >> v_2_0 >> v_3_0 >> v_4_0 >> v_5_0;
    std::cin >> a_0_0 >> a_1_0 >> a_2_0 >> a_3_0 >> a_4_0 >> a_5_0;

    auto res = FK_Indy7_fixed32_no_conversion(
        qcos_0_0, qcos_1_0, qcos_2_0, qcos_3_0, qcos_4_0, qcos_5_0,
        qsin_0_0, qsin_1_0, qsin_2_0, qsin_3_0, qsin_4_0, qsin_5_0,
        v_0_0, v_1_0, v_2_0, v_3_0, v_4_0, v_5_0,
        a_0_0, a_1_0, a_2_0, a_3_0, a_4_0, a_5_0
    );

    for(int i = 0; i < NU; i++){
        std::cout << "oMis[" << i << "].rotation: ";
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                std::cout << res.oMis[i].rotation[j][k] << " ";
            }
        }
        std::cout << "\noMis[" << i << "].translation: ";
        for(int j = 0; j < 3; j++){
            std::cout << res.oMis[i].translation[j] << " ";
        }
        std::cout << "\nv: ";
        for(int j = 0; j < 6; j++){
            std::cout << res.v[i][j] << " ";
        }
        std::cout << "\na: ";
        for(int j = 0; j < 6; j++){
            std::cout << res.a[i][j] << " ";
        }
        std::cout << "\n";
    }

    return 0;
}