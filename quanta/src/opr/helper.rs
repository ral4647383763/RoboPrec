use std::sync::Mutex;
use once_cell::sync::Lazy;


// Global counter for generating unique names.
static TEMP_VAR_COUNTER: Lazy<Mutex<i32>> = Lazy::new(|| Mutex::new(0));

// list of all the names generated
static TEMP_VAR_NAMES: Lazy<Mutex<Vec<String>>> = Lazy::new(|| Mutex::new(Vec::new()));

/// Generates a unique variable name given a prefix.
pub fn generate_name(prefix: &str) -> String {
    let mut counter = TEMP_VAR_COUNTER.lock().unwrap();
    *counter += 1;
    if prefix.is_empty() {
        return format!("r_{}", *counter);
    }
    else{
        format!("r_{}_{}", *counter, prefix)
    }
}

pub fn check_if_name_exists(name: &str) -> bool {
    let names = TEMP_VAR_NAMES.lock().unwrap();
    names.contains(&name.to_string())
}

pub fn add_name(name: &str) {
    let mut names = TEMP_VAR_NAMES.lock().unwrap();
    names.push(name.to_string());
}

/*
pub fn clear_name(name: &str) {
    let mut names = TEMP_VAR_NAMES.lock().unwrap();
    names.retain(|n| n != name);
}
*/
