# C++ Motion Planning Examples

cmake -S . -B build

cmake --build build -j

./build/plan config.yaml

python plot_c_space.py --config config.yaml

If planning with the task space Ceres also visualize with:

python plot_task_space.py --config config.yaml
