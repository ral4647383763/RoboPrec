import os
import time
import glob

# this file will delete ../daisy/ranges.txt and ../daisy/errors.txt
# then, you will run ./../daisy/daisy --codegen --lang=C --precision=Fixed32 --rangeMethod=interval --errorMethod=interval {file_path}
# You need to run this command for every scala file in generated/Scala folder

# You can just do that, but you need to run it for every precision, and we know it will fail for some precisions
#precisions = ["Fixed16", "Fixed32", "Float32", "Float64"]

# this is faster, it runs only the precisions that we know will work
uniform_precisions = {
    "forward_kinematics": {
        "4dof": ["Fixed16", "Fixed32", "Float32", "Float64", "Fixed4-28"],
        "5dof": ["Fixed16", "Fixed32", "Float32", "Float64", "Fixed5-27"],
        "6dof": ["Fixed16", "Fixed32", "Float32", "Float64", "Fixed6-26"],
        "7dof": ["Fixed16", "Fixed32", "Float32", "Float64", "Fixed7-25"]
    },
    "rnea": {
        "4dof": ["Fixed16", "Fixed32", "Float32", "Float64", "Fixed6-26"],
        "5dof": ["Fixed16", "Fixed32", "Float32", "Float64", "Fixed7-25"],
        "6dof": ["Fixed16", "Fixed32", "Float32", "Float64", "Fixed13-19"],
        "7dof": ["Fixed16", "Fixed32", "Float32", "Float64", "Fixed14-18"]
    },
    "rnea_derivatives": {
        "4dof": ["Fixed16", "Fixed32", "Float32", "Float64"],
        "5dof": [           "Fixed32", "Float32", "Float64"],
        "6dof": [           "Fixed32", "Float32", "Float64"],
        "7dof": [                      "Float32", "Float64"]
    },
}


def run_daisy(file_path, algorithm, dof, precision):

    # get the file list in ../daisy/output
    file_list = glob.glob("../daisy/output/*")
    print("command: ", f"cd ../daisy && ./daisy --codegen --lang=C --apfixed --precision={precision} --rangeMethod=interval --errorMethod=interval ../quanta/{file_path}")
    os.system(f"cd ../daisy && ./daisy --codegen --lang=C --apfixed --precision={precision} --rangeMethod=interval --errorMethod=interval ../quanta/{file_path}")
    file_list_new = glob.glob("../daisy/output/*")

    time.sleep(1.0)

    # find the new file
    new_file = None
    for file in file_list_new:
        if file not in file_list:
            new_file = file
            break
    
    # copy the new file to the correct location
    cp_command = f"cp {new_file} ../quanta/generated/C/{algorithm}/{dof}/{precision}_ap_fixed.cpp"
    mkdir_command = f"mkdir -p ../quanta/generated/C/{algorithm}/{dof}/"

    os.system(mkdir_command)
    os.system(cp_command)

    # remove the old file
    os.system(f"rm {new_file}")

    os.system("rm ../daisy/ranges.txt")
    os.system("rm ../daisy/errors.txt")
    os.system(f"cd ../daisy && ./daisy --codegen --lang=C --precision={precision} --rangeMethod=interval --errorMethod=interval ../quanta/{file_path}")
    print("command: ", f"cd ../daisy && ./daisy --codegen --lang=C --precision={precision} --rangeMethod=interval --errorMethod=interval ../quanta/{file_path}")

    time.sleep(1.0)

    file_list_new = glob.glob("../daisy/output/*")

    # find the new file
    new_file = None
    for file in file_list_new:
        if file not in file_list:
            new_file = file
            break

    # copy the new file to the correct location
    cp_command = f"cp {new_file} ../quanta/generated/C/{algorithm}/{dof}/{precision}_C.cpp"
    mkdir_command = f"mkdir -p ../quanta/generated/C/{algorithm}/{dof}/"

    os.system(mkdir_command)
    os.system(cp_command)

    # remove the old file
    print("rm file: ", new_file)
    os.system(f"rm {new_file}")

    # need to save range and errors
    cp_command = f"cp ../daisy/ranges.txt ../quanta/data/range\&error/{algorithm}/{dof}/range.txt"
    mkdir_command = f"mkdir -p ../quanta/data/range\&error/{algorithm}/{dof}/"
    print("mkdir command: ", mkdir_command)
    os.system(mkdir_command)
    # create the directory if it does not exist
    # os.makedirs(os.path.dirname(cp_command), exist_ok=True)
    print("cp command: ", cp_command)
    os.system(cp_command)
    print("command: ", cp_command)

    cp_command = f"cp ../daisy/errors.txt ../quanta/data/range\&error/{algorithm}/{dof}/"
    mkdir_command = f"mkdir -p ../quanta/data/range\&error/{algorithm}/{dof}/"
    # error copy
    match precision:
        case "Fixed16":
            cp_command += "int16/error.txt"
            mkdir_command += "int16/"
        case "Fixed32":
            cp_command += "int32/error.txt"
            mkdir_command += "int32/"
        case "Float32":
            cp_command += "float/error.txt"
            mkdir_command += "float/"
        case "Float64":
            cp_command += "double/error.txt"
            mkdir_command += "double/"
        case _:
            cp_command += "int32-uniform/error.txt"
            mkdir_command += "int32-uniform/"
    print("mkdir command: ", mkdir_command)
    os.system(mkdir_command)
    print("cp command: ", cp_command)
    os.system(cp_command)
    print("command: ", cp_command)

    # need to save the generated file
    # save to generated/C/{algorithm}/{dof}/{range}/{precision}_{out}.cpp
    # match out:
    #     case "C":
    #         cp_command = f"cp ../daisy/generated.cpp ../quanta/generated/C/{algorithm}/{dof}/{range}/{precision}_{out}.cpp"
    #     case "ap_fixed":
    #         cp_command = f"cp ../daisy/generated.cpp ../quanta/generated/ap_fixed/{algorithm}/{dof}/{range}/{precision}_{out}.cpp"
    # mkdir_command = f"mkdir -p ../quanta/generated/{out}/{algorithm}/{dof}/{range}/"
    # os.system(mkdir_command)




# iterate over all files recursively
# first folders are the algorithm name
# the second folders are the number of degrees of freedom
# then, the files are range types, large-range.scala and mid-range.scala etc.
# iterate over all files recursively, while saving the current algorithm, dof and range
# run daisy for each file

# read first set of folders
algorithm_folders = glob.glob('generated/Scala/*/')
print("algorithm_folders: ", algorithm_folders)
algorithms = [algorithm_folder.split('/')[-2] for algorithm_folder in algorithm_folders]
print("algorithms: ", algorithms)

# create /daisy/output if it does not exist
if not os.path.exists('/daisy/output'):
    os.makedirs('../daisy/output')



for algorithm_folder in algorithm_folders:
    algorithm = algorithm_folder.split('/')[-2]
    # Find all files ending with .scala
    dof_files = glob.glob(os.path.join(algorithm_folder, '*.scala'))
    print("dof_files: ", dof_files)

    # Process each file
    for dof_file in dof_files:
        # Extract the base name (e.g., '4dof') from the file path
        filename = os.path.basename(dof_file)
        dof = os.path.splitext(filename)[0]
        print("dof: ", dof)

        # also check for uniform precisions
        if algorithm in uniform_precisions and dof in uniform_precisions[algorithm]:
            precisions = uniform_precisions[algorithm][dof]
            for precision in precisions:
                run_daisy(dof_file, algorithm, dof, precision)








"""
for file_path in glob.iglob('generated/Scala/**/*.scala', recursive=True):
    for precision in precisions:
        for out in output:
            os.system("rm ../daisy/ranges.txt")
            os.system("rm ../daisy/errors.txt")
            command = "" 
            if out == "C":
                command = f"cd ../daisy && ./daisy --codegen --lang=C --precision={precision} --rangeMethod=interval --errorMethod=interval ../quanta/{file_path}"
            else:
                command = f"cd ../daisy && ./daisy --codegen --lang=C --apfixed --precision={precision} --rangeMethod=interval --errorMethod=interval ../quanta/{file_path}"
            os.system(command)
            print("command: ", command)

            # need to save range and errors
            cp_command = f"cp ../daisy/ranges.txt ../quanta/data/range&error/{file_path.split('/')[-1].replace('.scala', f'_{precision}_{out}_range.txt')}"
            os.system(cp_command)
            print("command: ", cp_command)
    #run_daisy(file_path)
"""
    






