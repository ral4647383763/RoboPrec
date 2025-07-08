import os


# run cargo install --path . --root .
os.system("cargo install --path . --root .")

# Then, 
#dofs = [4, 5, 6, 7]
algorithms = ["forward-kinematics", "rnea", "rnea-derivatives"]
dofs = [4, 5, 6, 7]
#algorithms = ["rnea-derivatives"]


# execute ./bin/quanta --dof {dof} --algorithm {algorithm} > {output_file}
for dof in dofs:
    for algorithm in algorithms:
        output_file = f"generated/Scala/{algorithm.replace('-','_')}/{dof}dof.scala"
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.exists(output_dir):
            print(f"Creating directory: {output_dir}")
            os.makedirs(output_dir, exist_ok=True)
        # create the directory if it does not exist
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        print(f"Executing: ./bin/quanta --dof {dof} --algorithm {algorithm} > {output_file}")
        os.system(f"./bin/quanta --dof {dof} --algorithm {algorithm} > {output_file}")
        print(f"Execution completed for: {output_file}")


