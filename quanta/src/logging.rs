use std::sync::Mutex;
use once_cell::sync::Lazy;
use crate::types::alias::{Scalar, Vector, Matrix};

#[macro_export]
macro_rules! my_println {
    () => {
        $crate::logging::log_println(format_args!(""))
    };
    ($($arg:tt)*) => {
        $crate::logging::log_println(format_args!($($arg)*))
    }
}

#[macro_export]
macro_rules! my_print {
    () => {
        $crate::logging::log_print(format_args!(""))
    };
    ($($arg:tt)*) => {
        $crate::logging::log_print(format_args!($($arg)*))
    }
}

/// Configuration for the function
pub static FUNCTION_NAME: Mutex<String> = Mutex::new(String::new());
pub static FUNCTION_INPUTS: Mutex<Vec<String>> = Mutex::new(Vec::new()); // This should also get vectors and matrices, but for now we only support scalars
pub static FUNCTION_OUTPUT: Mutex<Vec<String>> = Mutex::new(Vec::new()); // This will contain one output for now, as Daisy does not support multiple outputs yet
pub static FUNCTION_INPUT_RANGES: Mutex<Vec<(f64, f64)>> = Mutex::new(Vec::new()); // This will contain the ranges for each input

#[allow(dead_code)]
pub fn add_function_input_scalar(input: Scalar, range: (f64, f64)) {
    let mut input_guard = FUNCTION_INPUTS.lock().unwrap();
    input_guard.push(format!("{}_0_0", input.get_name()));
    let mut range_guard = FUNCTION_INPUT_RANGES.lock().unwrap();
    range_guard.push(range);
}

pub fn add_function_input_vector<const N: usize>(input: Vector<N>, range: Vec<(f64, f64)>) {
    let mut input_guard = FUNCTION_INPUTS.lock().unwrap();
    for i in 0..N {
        input_guard.push(format!("{}_{}_{}", input.get_name(), i, 0));
    }
    let mut range_guard = FUNCTION_INPUT_RANGES.lock().unwrap();
    for r in range {
        range_guard.push(r);
    }
}

#[allow(dead_code)]
pub fn add_function_input_matrix<const NROWS: usize, const NCOLS: usize>(input: Matrix<NROWS, NCOLS>, range: Vec<(f64, f64)>) {
    let mut input_guard = FUNCTION_INPUTS.lock().unwrap();
    for i in 0..NROWS {
        for j in 0..NCOLS {
            input_guard.push(format!("{}_{}_{}", input.get_name(), i, j));
        }
    }
    let mut range_guard = FUNCTION_INPUT_RANGES.lock().unwrap();
    for r in range {
        range_guard.push(r);
    }
}

pub fn set_function_name(name: &str) {
    let mut name_guard = FUNCTION_NAME.lock().unwrap();
    *name_guard = name.to_string();
}

pub fn set_function_output(output: Scalar) {
    let mut output_guard = FUNCTION_OUTPUT.lock().unwrap();
    output_guard.clear(); // Clear previous outputs
    output_guard.push(format!("{}_0_0", output.get_name()));
}

pub fn print_function() {
    let name_guard = FUNCTION_NAME.lock().unwrap();
    let inputs_guard = FUNCTION_INPUTS.lock().unwrap();

    println!("import daisy.lang._");
    println!("import Real._");
    println!();

    // use function name
    println!("object {} {{", name_guard);
    println!("def {}(", name_guard);

    // Print inputs
    for (i, input) in inputs_guard.iter().enumerate() {
        if i > 0 {
            print!(", ");
        }
        print!("{}: Real", input);
    }
    println!("): Real = {{");
    println!("require(");
    
    // print input ranges
    let len = FUNCTION_INPUT_RANGES.lock().unwrap().len();
    if len != inputs_guard.len() {
        panic!("The number of input ranges does not match the number of inputs. Please check your input ranges.");
    }
    for (i, range) in FUNCTION_INPUT_RANGES.lock().unwrap().iter().enumerate() {
        print!("{} > {:.} && {} < {:.} ", inputs_guard[i], range.0, inputs_guard[i], range.1);
        // if reached the last input, do not print the last '&&'
        if i != len - 1 {
            print!(" && ");
        }
    }
    println!("\n)");

    println!("{}", GENERATED_CODE.lock().unwrap().as_str());

    // Print output
    let output_guard = FUNCTION_OUTPUT.lock().unwrap();
    if output_guard.len() == 0 {
        panic!("No output set for the function. Use set_function_output() to set the output.");
    }
    println!("{}", output_guard[0]);
    
    println!("}}}}");

}



/// Configuration for logging.
pub struct LogConfig {
    /// When false, logging messages are suppressed.
    pub enabled: bool,
    // Optional custom output. If None, stdout is used.
    //pub output: Option<Box<dyn Write + Send>>,
}

impl Default for LogConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            //output: None,
        }
    }
}

/// Global logging configuration.
pub static LOG_CONFIG: Lazy<Mutex<LogConfig>> = Lazy::new(|| Mutex::new(LogConfig::default()));

/// Global variable holds the generated code before the function definition
/// This is very bad code, I need to change it!
pub static GENERATED_CODE: Lazy<Mutex<String>> = Lazy::new(|| Mutex::new(String::new()));

/// Prints a line with a trailing newline.
/// It accepts a preformatted std::fmt::Arguments.
pub fn log_println(args: std::fmt::Arguments) {
    log_print(args);
    // print newline
    log_print(format_args!("\n"));
}

/// Prints without a trailing newline.
/// It accepts a preformatted std::fmt::Arguments.
pub fn log_print(args: std::fmt::Arguments) {
    let config = LOG_CONFIG.lock().unwrap();
    if !config.enabled {
        return;
    }
    //if let Some(ref mut out) = config.output {
    //    let _ = write!(out, "{}", args);
    //    let _ = out.flush();
    //} else {
    //    print!("{}", args);
    //}
    // For now, instead, just append to GENERATED_CODE
    let mut generated_code = GENERATED_CODE.lock().unwrap();
    if generated_code.is_empty() {
        generated_code.push_str(&format!("{}", args));
    } else {
        generated_code.push_str(&format!(" {}", args));
    }
}

/*
/// Disable logging.
pub fn disable_dsl_print() {
    let mut config = LOG_CONFIG.lock().unwrap();
    config.enabled = false;
}

/// Enable logging.
pub fn enable_dsl_print() {
    let mut config = LOG_CONFIG.lock().unwrap();
    config.enabled = true;
}

/// Set a custom output target (for example, a file).
pub fn set_output<T>(target: T)
where
    T: Write + Send + 'static,
{
    let mut config = LOG_CONFIG.lock().unwrap();
    config.output = Some(Box::new(target));
}

/// Clear the custom output so that logging reverts to stdout.
pub fn clear_output() {
    let mut config = LOG_CONFIG.lock().unwrap();
    config.output = None;
}
*/