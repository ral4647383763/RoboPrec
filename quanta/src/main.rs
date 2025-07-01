mod opr;
mod types;
mod logging;
mod robots;
mod examples;
mod paper_tests;

#[cfg(test)]
mod tests;

use paper_tests::paper_tests;
use examples::ex1::ex1;
use examples::ex2::ex2;
use examples::ex3::ex3;
use examples::ex_loop::ex_loop;


fn main() {
    paper_tests();

    //ex1();
    //ex2();
    //ex3();
    //ex_loop();
}

