#include <math.h>
#include <iostream>
#include <benchmark/benchmark.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

#define NUM_INPUT_SETS 1000 // Number of input sets to read from the CSV

// Structure to hold one complete set of inputs and precalculated sin/cos
struct InputSet {
    std::vector<double> q;
    std::vector<double> v;
    std::vector<double> a;
    // Pre-calculated sin/cos for efficiency in the benchmark loop
    std::vector<double> q_sin;
    std::vector<double> q_cos;
};


// Function to load all input sets from the CSV file
std::vector<InputSet> load_input_sets_from_csv(const std::string& filename) {
    std::vector<InputSet> all_sets;
    all_sets.reserve(NUM_INPUT_SETS);
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        throw std::runtime_error("Error: Could not open input file: " + filename);
    }

    // Optional: Skip header line if it exists (simple check for '#')
    if (std::getline(file, line) && line[0] == '#') {
        // Header skipped
    } else {
        // No header or first line is data, rewind/reset stream state if needed
        // For simplicity, we'll assume if it doesn't start with '#', it's data
        // If the first line was read, process it now. If getline failed, loop below won't run.
        if (file.good() && !line.empty()) {
             // Process the line read by the header check
             goto process_line; // Use goto for simplicity here, alternatives exist
        }
    }


    while (std::getline(file, line)) {
process_line: // Label for processing the line
        if (line.empty() || line[0] == '#') continue; // Skip empty lines or comments

        std::stringstream ss(line);
        std::string segment;
        InputSet current_set;
        current_set.q.resize(NU);
        current_set.v.resize(NU);
        current_set.a.resize(NU);
        current_set.q_sin.resize(NU);
        current_set.q_cos.resize(NU);

        try {
            // Read q values
            for (int i = 0; i < NU; ++i) {
                if (!std::getline(ss, segment, ',')) throw std::runtime_error("Insufficient columns");
                current_set.q[i] = std::stod(segment);
                // Pre-calculate sin/cos during load
                current_set.q_sin[i] = std::sin(current_set.q[i]);
                current_set.q_cos[i] = std::cos(current_set.q[i]);
            }
            // Read v values
            for (int i = 0; i < NU; ++i) {
                if (!std::getline(ss, segment, ',')) throw std::runtime_error("Insufficient columns");
                current_set.v[i] = std::stod(segment);
            }
            // Read a values
            for (int i = 0; i < NU; ++i) {
                 // Last element might not have a trailing comma
                if (!std::getline(ss, segment, (i == NU - 1) ? '\n' : ',')) {
                     // If it's the last element and getline fails on '\n', check ss directly
                     if (i == NU-1 && !segment.empty()) {
                         // Process segment read before failure (if any)
                     } else {
                        throw std::runtime_error("Insufficient columns or format error");
                     }
                }
                current_set.a[i] = std::stod(segment);
            }

            if (all_sets.size() < NUM_INPUT_SETS) {
                 all_sets.push_back(std::move(current_set));
            } else {
                // Stop if we have already read the desired number of sets
                break;
            }

        } catch (const std::invalid_argument& e) {
            throw std::runtime_error("Error parsing number in CSV line: " + line + " (" + e.what() + ")");
        } catch (const std::out_of_range& e) {
             throw std::runtime_error("Error parsing number (out of range) in CSV line: " + line + " (" + e.what() + ")");
        } catch (const std::runtime_error& e) {
             throw std::runtime_error("Error reading line: " + line + " (" + e.what() + ")");
        }
         line.clear(); // Clear line in case goto was used
    }

    if (all_sets.size() != NUM_INPUT_SETS) {
         std::cerr << "Warning: Expected " << NUM_INPUT_SETS << " data sets, but read " << all_sets.size() << " from " << filename << std::endl;
         if (all_sets.empty()) {
             throw std::runtime_error("Error: No valid data sets loaded from file: " + filename);
         }
    }

    return all_sets;
}