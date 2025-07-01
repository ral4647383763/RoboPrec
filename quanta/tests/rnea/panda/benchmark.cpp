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
#include "fixed14_18.hpp"
#include "fixed16.hpp"

#include "../../util/benchmark_utils.hpp"

static std::vector<InputSet> input_sets = load_input_sets_from_csv("../../inputs.csv");


static void BK_RNEA_Panda_double(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    for (auto _ : state) {
        // Cycle through the input set
        auto& current_set = input_sets[idx++];
        if (idx == num_loaded_sets) idx = 0;
        
        auto res = RNEA_Panda_double(
            current_set.q_cos[0], current_set.q_cos[1], current_set.q_cos[2], current_set.q_cos[3], current_set.q_cos[4], current_set.q_cos[5], current_set.q_cos[6],
            current_set.q_sin[0], current_set.q_sin[1], current_set.q_sin[2], current_set.q_sin[3], current_set.q_sin[4], current_set.q_sin[5], current_set.q_sin[6],
            current_set.v[0], current_set.v[1], current_set.v[2], current_set.v[3], current_set.v[4], current_set.v[5], current_set.v[6],
            current_set.a[0], current_set.a[1], current_set.a[2], current_set.a[3], current_set.a[4], current_set.a[5], current_set.a[6]
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }

}

static void BK_RNEA_Panda_float(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    for (auto _ : state) {
        // Cycle through the input set
        auto& current_set = input_sets[idx++];
        if (idx == num_loaded_sets) idx = 0;
        
        auto res = RNEA_Panda_float(
            current_set.q_cos[0], current_set.q_cos[1], current_set.q_cos[2], current_set.q_cos[3], current_set.q_cos[4], current_set.q_cos[5], current_set.q_cos[6],
            current_set.q_sin[0], current_set.q_sin[1], current_set.q_sin[2], current_set.q_sin[3], current_set.q_sin[4], current_set.q_sin[5], current_set.q_sin[6],
            current_set.v[0], current_set.v[1], current_set.v[2], current_set.v[3], current_set.v[4], current_set.v[5], current_set.v[6],
            current_set.a[0], current_set.a[1], current_set.a[2], current_set.a[3], current_set.a[4], current_set.a[5], current_set.a[6]
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }

}

static void BK_RNEA_Panda_fixed32(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    for (auto _ : state) {
        // Cycle through the input set
        auto& current_set = input_sets[idx++];
        if (idx == num_loaded_sets) idx = 0;
        
        auto res = RNEA_Panda_fixed32(
            current_set.q_cos[0], current_set.q_cos[1], current_set.q_cos[2], current_set.q_cos[3], current_set.q_cos[4], current_set.q_cos[5], current_set.q_cos[6],
            current_set.q_sin[0], current_set.q_sin[1], current_set.q_sin[2], current_set.q_sin[3], current_set.q_sin[4], current_set.q_sin[5], current_set.q_sin[6],
            current_set.v[0], current_set.v[1], current_set.v[2], current_set.v[3], current_set.v[4], current_set.v[5], current_set.v[6],
            current_set.a[0], current_set.a[1], current_set.a[2], current_set.a[3], current_set.a[4], current_set.a[5], current_set.a[6]
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }
}

static void BK_RNEA_Panda_fixed14_18(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    for (auto _ : state) {
        // Cycle through the input set
        auto& current_set = input_sets[idx++];
        if (idx == num_loaded_sets) idx = 0;
        
        auto res = RNEA_Panda_fixed14_18(
            current_set.q_cos[0], current_set.q_cos[1], current_set.q_cos[2], current_set.q_cos[3], current_set.q_cos[4], current_set.q_cos[5], current_set.q_cos[6],
            current_set.q_sin[0], current_set.q_sin[1], current_set.q_sin[2], current_set.q_sin[3], current_set.q_sin[4], current_set.q_sin[5], current_set.q_sin[6],
            current_set.v[0], current_set.v[1], current_set.v[2], current_set.v[3], current_set.v[4], current_set.v[5], current_set.v[6],
            current_set.a[0], current_set.a[1], current_set.a[2], current_set.a[3], current_set.a[4], current_set.a[5], current_set.a[6]
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }
}

static void BK_RNEA_Panda_fixed16(benchmark::State& state) {
    const size_t num_loaded_sets = input_sets.size();
    static size_t idx = 0;

    for (auto _ : state) {
        // Cycle through the input set
        auto& current_set = input_sets[idx++];
        if (idx == num_loaded_sets) idx = 0;
        
        auto res = RNEA_Panda_fixed16(
            current_set.q_cos[0], current_set.q_cos[1], current_set.q_cos[2], current_set.q_cos[3], current_set.q_cos[4], current_set.q_cos[5], current_set.q_cos[6],
            current_set.q_sin[0], current_set.q_sin[1], current_set.q_sin[2], current_set.q_sin[3], current_set.q_sin[4], current_set.q_sin[5], current_set.q_sin[6],
            current_set.v[0], current_set.v[1], current_set.v[2], current_set.v[3], current_set.v[4], current_set.v[5], current_set.v[6],
            current_set.a[0], current_set.a[1], current_set.a[2], current_set.a[3], current_set.a[4], current_set.a[5], current_set.a[6]
        );
    
        // Use the result to prevent compiler optimizations
        benchmark::DoNotOptimize(res);

        benchmark::ClobberMemory();
    }
}


BENCHMARK(BK_RNEA_Panda_double)->MinWarmUpTime(3.0)->MinTime(10.0);
BENCHMARK(BK_RNEA_Panda_float)->MinWarmUpTime(3.0)->MinTime(10.0);
BENCHMARK(BK_RNEA_Panda_fixed32)->MinWarmUpTime(3.0)->MinTime(10.0);
BENCHMARK(BK_RNEA_Panda_fixed14_18)->MinWarmUpTime(3.0)->MinTime(10.0);
BENCHMARK(BK_RNEA_Panda_fixed16)->MinWarmUpTime(3.0)->MinTime(10.0);

BENCHMARK_MAIN();

