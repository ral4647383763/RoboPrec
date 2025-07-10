# RoboPrec

This document provides instructions on how to set up the environment and run the examples and experiments presented in the paper.

---

## **Prerequisites**

You must have **Docker** installed on your system to run these experiments.


## **Getting Started**

1.  **Build the Docker Image**
    Open a terminal in the project's root directory and run the following command to build the Docker image. Replace `{image_name}` with a name of your choice (e.g., `roboprec-artifact`).

    ```bash
    docker build . -t {image_name}
    ```

2.  **Run the Docker Container**
    Once the image is built, start an interactive session inside the container:

    ```bash
    docker run --name {container_name} -i -t {image_name} bash
    ```
    *Note: The container runs a snapshot of the code from build time. It does not mount your local directories. If you wish to work with your local code, you'll need to mount the directory using the `-v` flag.*

---

## **Running an Example**

Follow these steps to generate and analyze a single example.

1.  **Select an Example**
    Navigate to the `quanta` source directory:
    ```bash
    cd /quanta/src
    ```
    Open `main.rs` in an editor (e.g., `vim`, `nano`). The file is configured to run the paper experiments by default. To run a standalone example, comment out the `paper_tests();` line and uncomment one of the example lines (e.g., `ex1::run();`). The implementations for these examples can be found in `/quanta/src/examples/`.

2.  **Generate Code/Apply Analysis**
    Run in /quanta directory:
    ```bash
    python3 scripts/run_one_codegen.py {codegen_type} {precision}
    ```
    {codegen_type} can be either "C" or "ap_fixed", and precision can be "Float16, Float32, Float64, Fixed{1-64}, Fixed{1-64}-{1-64}". For example, if HLS-friendly code is wanted and Float64 is wanted, you can run:
    ```bash
    python3 scripts/run_one_codegen.py ap_fixed Float64
    ```
    For C code and fixed-point uniform precision where the integer bits are 48 and fractional bits are 16,
    ```bash
    python3 scripts/run_one_codegen.py C Fixed48-16
    ```
    For HLS-friendly code and mixed-precision fixed-point, where the total number of bits are 32:
    ```bash
    python3 scripts/run_one_codegen.py ap_fixed Fixed32
    ```

    The analysis bounds will be printed to the console, and the generated C code will be available in the `/quanta/output` directory.

It should be easy to implement your own examples. Every operation and every feature of the language is permissible, as long as you do not use a program that has a dynamic dataflow. 

---

## **Reproducing Paper Results**

To generate the code for all algorithms and robots presented in the paper, follow these steps.

1.  **Enable Paper Experiments**
    In `/quanta/src/main.rs`, ensure only the `paper_tests();` line is uncommented.

2.  **Worst case errors plot**
    From the `/quanta` directory, execute:
    ```bash
    # Make sure you are in the /quanta directory
    python3 scripts/run_all_codegen.py
    python3 scripts/run_daisy.py
    python3 scripts/error_range_process.py
    ```
3.  **Average case plots**
    From the `/quanta` directory, execute:
    ```bash
    # Make sure you are in the /quanta directory
    ./avg_case_tests.sh
    ```
    > **Warning** ⚠️
    > The scripts to reproduce the plots are available, but should be run with caution. The `worst-case-error` plot, for instance, takes approximately **9 hours** to complete.

4. **Runtime plot**
    From the `/quanta` directory, execute:
    ```bash
    # Make sure you are in the /quanta directory
    python3 scripts/bar_runtime_plot.py
    ```
    The plot will be in `/quanta/plots/`

5. **Running benchmarks**
    To run the benchmarks, see `quanta/tests/{algorithm}/{robot}/benchmark.cpp` files. To run them, first install Google Benchmark library, then compile the benchmark files with:
    ```bash
    g++ benchmark.cpp -std=c++17 -isystem ../../../../benchmark/include -L../../../../benchmark/build/src -lbenchmark -lpthread -o benchmark -O3 && ./benchmark
    ```
    You may change the include directory and library path according to your system setup. The results will be printed to the console.



---

