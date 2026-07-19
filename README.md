# motion_planning_examples

cmake -S . -B build

cmake --build build -j

./build/plan build/rrtstar_config.yaml

python plot_c_space.py --config build/rrtstar_config.yaml
