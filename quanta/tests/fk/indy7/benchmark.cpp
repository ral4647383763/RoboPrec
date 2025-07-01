#include <math.h>
#include <iostream>
#include <benchmark/benchmark.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

#include "double.hpp"
#include "float.hpp"
#include "fixed32.hpp"
#include "fixed6-26.hpp"
#include "fixed16.hpp"

#include "float_no_conversion.hpp"
#include "fixed32_no_conversion.hpp"
#include "fixed6-26_no_conversion.hpp"
#include "fixed16_no_conversion.hpp"

#include "../../util/benchmark_utils.hpp"

static std::vector<InputSet> input_sets = load_input_sets_from_csv("../../inputs.csv");


static void BK_FK_Indy7_double(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;
    // Cycle through the input set
    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;
    double q_0 = current_set.q[0];
    double q_1 = current_set.q[1];
    double q_2 = current_set.q[2];
    double q_3 = current_set.q[3];
    double q_4 = current_set.q[4];
    double q_5 = current_set.q[5];
    double v_0 = current_set.v[0];
    double v_1 = current_set.v[1];
    double v_2 = current_set.v[2];
    double v_3 = current_set.v[3];
    double v_4 = current_set.v[4];
    double v_5 = current_set.v[5];
    double a_0 = current_set.a[0];
    double a_1 = current_set.a[1];
    double a_2 = current_set.a[2];
    double a_3 = current_set.a[3];
    double a_4 = current_set.a[4];
    double a_5 = current_set.a[5];

    for (auto _ : state) {
        
        auto res = FK_Indy7_double(
            cos(q_0), cos(q_1), cos(q_2), cos(q_3), cos(q_4), cos(q_5),
            sin(q_0), sin(q_1), sin(q_2), sin(q_3), sin(q_4), sin(q_5),
            v_0, v_1, v_2, v_3, v_4, v_5,
            a_0, a_1, a_2, a_3, a_4, a_5
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }

}

static void BK_FK_Indy7_float(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;
    // Cycle through the input set
    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;

    double q_0 = current_set.q[0];
    double q_1 = current_set.q[1];
    double q_2 = current_set.q[2];
    double q_3 = current_set.q[3];
    double q_4 = current_set.q[4];
    double q_5 = current_set.q[5];
    double v_0 = current_set.v[0];
    double v_1 = current_set.v[1];
    double v_2 = current_set.v[2];
    double v_3 = current_set.v[3];
    double v_4 = current_set.v[4];
    double v_5 = current_set.v[5];
    double a_0 = current_set.a[0];
    double a_1 = current_set.a[1];
    double a_2 = current_set.a[2];
    double a_3 = current_set.a[3];
    double a_4 = current_set.a[4];
    double a_5 = current_set.a[5];

    for (auto _ : state) {
        
        auto res = FK_Indy7_float(
            cos(q_0), cos(q_1), cos(q_2), cos(q_3), cos(q_4), cos(q_5),
            sin(q_0), sin(q_1), sin(q_2), sin(q_3), sin(q_4), sin(q_5),
            v_0, v_1, v_2, v_3, v_4, v_5,
            a_0, a_1, a_2, a_3, a_4, a_5
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }

}


static void BK_FK_Indy7_float_no_conversion(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;

    float q_0 = current_set.q[0];
    float q_1 = current_set.q[1];
    float q_2 = current_set.q[2];
    float q_3 = current_set.q[3];
    float q_4 = current_set.q[4];
    float q_5 = current_set.q[5];

    float v_0 = current_set.v[0];
    float v_1 = current_set.v[1];
    float v_2 = current_set.v[2];
    float v_3 = current_set.v[3];
    float v_4 = current_set.v[4];
    float v_5 = current_set.v[5];

    float a_0 = current_set.a[0];
    float a_1 = current_set.a[1];
    float a_2 = current_set.a[2];
    float a_3 = current_set.a[3];
    float a_4 = current_set.a[4];
    float a_5 = current_set.a[5];

    for (auto _ : state) {
        
        auto res = FK_Indy7_float_no_conversion(
            cosf(q_0), cosf(q_1), cosf(q_2), cosf(q_3), cosf(q_4), cosf(q_5),
            sinf(q_0), sinf(q_1), sinf(q_2), sinf(q_3), sinf(q_4), sinf(q_5),
            v_0, v_1, v_2, v_3, v_4, v_5,
            a_0, a_1, a_2, a_3, a_4, a_5
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }

}



static void BK_FK_Indy7_fixed32(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;
    // Cycle through the input set
    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;

    double q_0 = current_set.q[0];
    double q_1 = current_set.q[1];
    double q_2 = current_set.q[2];
    double q_3 = current_set.q[3];
    double q_4 = current_set.q[4];
    double q_5 = current_set.q[5];
    double v_0 = current_set.v[0];
    double v_1 = current_set.v[1];
    double v_2 = current_set.v[2];
    double v_3 = current_set.v[3];
    double v_4 = current_set.v[4];
    double v_5 = current_set.v[5];
    double a_0 = current_set.a[0];
    double a_1 = current_set.a[1];
    double a_2 = current_set.a[2];
    double a_3 = current_set.a[3];
    double a_4 = current_set.a[4];
    double a_5 = current_set.a[5];

    for (auto _ : state) {
        
        auto res = FK_Indy7_fixed32(
            cos(q_0), cos(q_1), cos(q_2), cos(q_3), cos(q_4), cos(q_5),
            sin(q_0), sin(q_1), sin(q_2), sin(q_3), sin(q_4), sin(q_5),
            v_0, v_1, v_2, v_3, v_4, v_5,
            a_0, a_1, a_2, a_3, a_4, a_5
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }
}

static void BK_FK_Indy7_fixed32_no_conversion(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    // Cycle through the input set
    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;
    
    double q_0 = current_set.q[0];
    double q_1 = current_set.q[1];
    double q_2 = current_set.q[2];
    double q_3 = current_set.q[3];
    double q_4 = current_set.q[4];
    double q_5 = current_set.q[5];

    int qcos_0_0 = (int)(current_set.q_cos[0] * pow(2, 30));
    int qcos_1_0 = (int)(current_set.q_cos[1] * pow(2, 30));
    int qcos_2_0 = (int)(current_set.q_cos[2] * pow(2, 30));
    int qcos_3_0 = (int)(current_set.q_cos[3] * pow(2, 30));
    int qcos_4_0 = (int)(current_set.q_cos[4] * pow(2, 30));
    int qcos_5_0 = (int)(current_set.q_cos[5] * pow(2, 30));

    int qsin_0_0 = (int)(current_set.q_sin[0] * pow(2, 30));
    int qsin_1_0 = (int)(current_set.q_sin[1] * pow(2, 30));
    int qsin_2_0 = (int)(current_set.q_sin[2] * pow(2, 30));
    int qsin_3_0 = (int)(current_set.q_sin[3] * pow(2, 30));
    int qsin_4_0 = (int)(current_set.q_sin[4] * pow(2, 30));
    int qsin_5_0 = (int)(current_set.q_sin[5] * pow(2, 30));

    int v_0_0 = (int)(current_set.v[0] * pow(2, 31));
    int v_1_0 = (int)(current_set.v[1] * pow(2, 31));
    int v_2_0 = (int)(current_set.v[2] * pow(2, 31));
    int v_3_0 = (int)(current_set.v[3] * pow(2, 31));
    int v_4_0 = (int)(current_set.v[4] * pow(2, 31));
    int v_5_0 = (int)(current_set.v[5] * pow(2, 31));

    int a_0_0 = (int)(current_set.a[0] * pow(2, 30));
    int a_1_0 = (int)(current_set.a[1] * pow(2, 30));
    int a_2_0 = (int)(current_set.a[2] * pow(2, 30));
    int a_3_0 = (int)(current_set.a[3] * pow(2, 30));
    int a_4_0 = (int)(current_set.a[4] * pow(2, 30));
    int a_5_0 = (int)(current_set.a[5] * pow(2, 30));

    for (auto _ : state) {
        
        auto res = FK_Indy7_fixed32_no_conversion(
            qcos_0_0, qcos_1_0, qcos_2_0, qcos_3_0, qcos_4_0, qcos_5_0,
            qsin_0_0, qsin_1_0, qsin_2_0, qsin_3_0, qsin_4_0, qsin_5_0,
            v_0_0, v_1_0, v_2_0, v_3_0, v_4_0, v_5_0,
            a_0_0, a_1_0, a_2_0, a_3_0, a_4_0, a_5_0
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);
        benchmark::DoNotOptimize(cos(q_0));
        benchmark::DoNotOptimize(sin(q_0));
        benchmark::DoNotOptimize(cos(q_1));
        benchmark::DoNotOptimize(sin(q_1));
        benchmark::DoNotOptimize(cos(q_2));
        benchmark::DoNotOptimize(sin(q_2));
        benchmark::DoNotOptimize(cos(q_3));
        benchmark::DoNotOptimize(sin(q_3));
        benchmark::DoNotOptimize(cos(q_4));
        benchmark::DoNotOptimize(sin(q_4));
        benchmark::DoNotOptimize(cos(q_5));
        benchmark::DoNotOptimize(sin(q_5));

        benchmark::ClobberMemory();
    }
}

static void BK_FK_Indy7_fixed6_26(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;
    // Cycle through the input set
    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;

    double q_0 = current_set.q[0];
    double q_1 = current_set.q[1];
    double q_2 = current_set.q[2];
    double q_3 = current_set.q[3];
    double q_4 = current_set.q[4];
    double q_5 = current_set.q[5];
    double v_0 = current_set.v[0];
    double v_1 = current_set.v[1];
    double v_2 = current_set.v[2];
    double v_3 = current_set.v[3];
    double v_4 = current_set.v[4];
    double v_5 = current_set.v[5];
    double a_0 = current_set.a[0];
    double a_1 = current_set.a[1];
    double a_2 = current_set.a[2];
    double a_3 = current_set.a[3];
    double a_4 = current_set.a[4];
    double a_5 = current_set.a[5];

    for (auto _ : state) {
        
        auto res = FK_Indy7_fixed6_26(
            cos(q_0), cos(q_1), cos(q_2), cos(q_3), cos(q_4), cos(q_5),
            sin(q_0), sin(q_1), sin(q_2), sin(q_3), sin(q_4), sin(q_5),
            v_0, v_1, v_2, v_3, v_4, v_5,
            a_0, a_1, a_2, a_3, a_4, a_5
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }
}


static void BK_FK_Indy7_fixed6_26_no_conversion(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    // Cycle through the input set
    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;

    double q_0 = current_set.q[0];
    double q_1 = current_set.q[1];
    double q_2 = current_set.q[2];
    double q_3 = current_set.q[3];
    double q_4 = current_set.q[4];
    double q_5 = current_set.q[5];

    int qcos_0_0 = (int)(current_set.q_cos[0] * pow(2, 26));
    int qcos_1_0 = (int)(current_set.q_cos[1] * pow(2, 26));
    int qcos_2_0 = (int)(current_set.q_cos[2] * pow(2, 26));
    int qcos_3_0 = (int)(current_set.q_cos[3] * pow(2, 26));
    int qcos_4_0 = (int)(current_set.q_cos[4] * pow(2, 26));
    int qcos_5_0 = (int)(current_set.q_cos[5] * pow(2, 26));

    int qsin_0_0 = (int)(current_set.q_sin[0] * pow(2, 26));
    int qsin_1_0 = (int)(current_set.q_sin[1] * pow(2, 26));
    int qsin_2_0 = (int)(current_set.q_sin[2] * pow(2, 26));
    int qsin_3_0 = (int)(current_set.q_sin[3] * pow(2, 26));
    int qsin_4_0 = (int)(current_set.q_sin[4] * pow(2, 26));
    int qsin_5_0 = (int)(current_set.q_sin[5] * pow(2, 26));

    int v_0_0 = (int)(current_set.v[0] * pow(2, 26));
    int v_1_0 = (int)(current_set.v[1] * pow(2, 26));
    int v_2_0 = (int)(current_set.v[2] * pow(2, 26));
    int v_3_0 = (int)(current_set.v[3] * pow(2, 26));
    int v_4_0 = (int)(current_set.v[4] * pow(2, 26));
    int v_5_0 = (int)(current_set.v[5] * pow(2, 26));

    int a_0_0 = (int)(current_set.a[0] * pow(2, 26));
    int a_1_0 = (int)(current_set.a[1] * pow(2, 26));
    int a_2_0 = (int)(current_set.a[2] * pow(2, 26));
    int a_3_0 = (int)(current_set.a[3] * pow(2, 26));
    int a_4_0 = (int)(current_set.a[4] * pow(2, 26));
    int a_5_0 = (int)(current_set.a[5] * pow(2, 26));

    for (auto _ : state) {
        
        auto res = FK_Indy7_fixed6_26_no_conversion(
            qcos_0_0, qcos_1_0, qcos_2_0, qcos_3_0, qcos_4_0, qcos_5_0,
            qsin_0_0, qsin_1_0, qsin_2_0, qsin_3_0, qsin_4_0, qsin_5_0,
            v_0_0, v_1_0, v_2_0, v_3_0, v_4_0, v_5_0,
            a_0_0, a_1_0, a_2_0, a_3_0, a_4_0, a_5_0
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);
        benchmark::DoNotOptimize(cos(q_0));
        benchmark::DoNotOptimize(sin(q_0));
        benchmark::DoNotOptimize(cos(q_1));
        benchmark::DoNotOptimize(sin(q_1));
        benchmark::DoNotOptimize(cos(q_2));
        benchmark::DoNotOptimize(sin(q_2));
        benchmark::DoNotOptimize(cos(q_3));
        benchmark::DoNotOptimize(sin(q_3));
        benchmark::DoNotOptimize(cos(q_4));
        benchmark::DoNotOptimize(sin(q_4));
        benchmark::DoNotOptimize(cos(q_5));
        benchmark::DoNotOptimize(sin(q_5));

        benchmark::ClobberMemory();
    }
}

static void BK_FK_Indy7_fixed16(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;
    // Cycle through the input set
    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;
    
    for (auto _ : state) {
        
        auto res = FK_Indy7_fixed16(
            current_set.q_cos[0], current_set.q_cos[1], current_set.q_cos[2], current_set.q_cos[3], current_set.q_cos[4], current_set.q_cos[5],
            current_set.q_sin[0], current_set.q_sin[1], current_set.q_sin[2], current_set.q_sin[3], current_set.q_sin[4], current_set.q_sin[5],
            current_set.v[0], current_set.v[1], current_set.v[2], current_set.v[3], current_set.v[4], current_set.v[5],
            current_set.a[0], current_set.a[1], current_set.a[2], current_set.a[3], current_set.a[4], current_set.a[5]
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }
}

static void BK_FK_Indy7_fixed16_no_conversion(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    // Cycle through the input set
    auto& current_set = input_sets[idx++];
    if (idx == num_loaded_sets) idx = 0;

    double q_0 = current_set.q[0];
    double q_1 = current_set.q[1];
    double q_2 = current_set.q[2];
    double q_3 = current_set.q[3];
    double q_4 = current_set.q[4];
    double q_5 = current_set.q[5];    

    int qcos_0_0 = (int)(current_set.q_cos[0] * pow(2, 14));
    int qcos_1_0 = (int)(current_set.q_cos[1] * pow(2, 14));
    int qcos_2_0 = (int)(current_set.q_cos[2] * pow(2, 14));
    int qcos_3_0 = (int)(current_set.q_cos[3] * pow(2, 14));
    int qcos_4_0 = (int)(current_set.q_cos[4] * pow(2, 14));
    int qcos_5_0 = (int)(current_set.q_cos[5] * pow(2, 14));

    int qsin_0_0 = (int)(current_set.q_sin[0] * pow(2, 14));
    int qsin_1_0 = (int)(current_set.q_sin[1] * pow(2, 14));
    int qsin_2_0 = (int)(current_set.q_sin[2] * pow(2, 14));
    int qsin_3_0 = (int)(current_set.q_sin[3] * pow(2, 14));
    int qsin_4_0 = (int)(current_set.q_sin[4] * pow(2, 14));
    int qsin_5_0 = (int)(current_set.q_sin[5] * pow(2, 14));

    int v_0_0 = (int)(current_set.v[0] * pow(2, 15));
    int v_1_0 = (int)(current_set.v[1] * pow(2, 15));
    int v_2_0 = (int)(current_set.v[2] * pow(2, 15));
    int v_3_0 = (int)(current_set.v[3] * pow(2, 15));
    int v_4_0 = (int)(current_set.v[4] * pow(2, 15));
    int v_5_0 = (int)(current_set.v[5] * pow(2, 15));

    int a_0_0 = (int)(current_set.a[0] * pow(2, 14));
    int a_1_0 = (int)(current_set.a[1] * pow(2, 14));
    int a_2_0 = (int)(current_set.a[2] * pow(2, 14));
    int a_3_0 = (int)(current_set.a[3] * pow(2, 14));
    int a_4_0 = (int)(current_set.a[4] * pow(2, 14));
    int a_5_0 = (int)(current_set.a[5] * pow(2, 14));

    for (auto _ : state) {
        // Cycle through the input set
        auto& current_set = input_sets[idx++];
        if (idx == num_loaded_sets) idx = 0;
        
        auto res = FK_Indy7_fixed16_no_conversion(
            qcos_0_0, qcos_1_0, qcos_2_0, qcos_3_0, qcos_4_0, qcos_5_0,
            qsin_0_0, qsin_1_0, qsin_2_0, qsin_3_0, qsin_4_0, qsin_5_0,
            v_0_0, v_1_0, v_2_0, v_3_0, v_4_0, v_5_0,
            a_0_0, a_1_0, a_2_0, a_3_0, a_4_0, a_5_0
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);
        benchmark::DoNotOptimize(cos(q_0));
        benchmark::DoNotOptimize(sin(q_0));
        benchmark::DoNotOptimize(cos(q_1));
        benchmark::DoNotOptimize(sin(q_1));
        benchmark::DoNotOptimize(cos(q_2));
        benchmark::DoNotOptimize(sin(q_2));
        benchmark::DoNotOptimize(cos(q_3));
        benchmark::DoNotOptimize(sin(q_3));
        benchmark::DoNotOptimize(cos(q_4));
        benchmark::DoNotOptimize(sin(q_4));
        benchmark::DoNotOptimize(cos(q_5));
        benchmark::DoNotOptimize(sin(q_5));

        benchmark::ClobberMemory();
    }
}

BENCHMARK(BK_FK_Indy7_double)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);
BENCHMARK(BK_FK_Indy7_float)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);
BENCHMARK(BK_FK_Indy7_fixed32)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);
BENCHMARK(BK_FK_Indy7_fixed6_26)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);
BENCHMARK(BK_FK_Indy7_fixed16)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);

BENCHMARK(BK_FK_Indy7_float_no_conversion)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);
BENCHMARK(BK_FK_Indy7_fixed32_no_conversion)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);
BENCHMARK(BK_FK_Indy7_fixed6_26_no_conversion)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);
BENCHMARK(BK_FK_Indy7_fixed16_no_conversion)->Iterations(100000)->Repetitions(10)->MinWarmUpTime(1.0)->ReportAggregatesOnly(false)->DisplayAggregatesOnly(false);

BENCHMARK_MAIN();

