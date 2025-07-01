g++ -O3 tests/fk/indy7/double_vs_float_main.cpp -o fk_float_main
g++ -O3 tests/fk/indy7/double_vs_fixed32_main.cpp -o fk_fixed32_main
g++ -O3 tests/rnea/indy7/double_vs_float_main.cpp -o rnea_float_main
g++ -O3 tests/rnea/indy7/double_vs_fixed32_main.cpp -o rnea_fixed32_main
g++ -O3 tests/rneaderiv/indy7/double_vs_float_main.cpp -o rneaderiv_float_main
g++ -O3 tests/rneaderiv/indy7/double_vs_fixed32_main.cpp -o rneaderiv_fixed32_main
./fk_float_main > fk_float.txt
./fk_fixed32_main > fk_fixed32.txt
./rnea_float_main > rnea_float.txt
./rnea_fixed32_main > rnea_fixed32.txt
./rneaderiv_float_main > rneaderiv_float.txt
./rneaderiv_fixed32_main > rneaderiv_fixed32.txt

python3 scripts/avg_case_cdf.py




