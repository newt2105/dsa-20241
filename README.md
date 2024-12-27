# Path Finding Algorithms

## Dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install build-essential cmake ninja-build graphviz
```

## Clone Repository

```bash
git clone https://github.com/newt2105/dsa-20241.git
```

## Compile and Run

* Before compiling, update the file path of data.txt in src/ultis.cpp to your absolute path.
* Stay in the root directory where CMakeLists.txt is located and run this command:

## Build

```bash
cmake --preset=cpp-x86_64-release
cd ./out/build/cpp-x86_64-release
ninja
```

### Run

```bash
./PathFinding
```

## Test

## Notes

* Ensure that all dependencies are correctly installed to avoid issues with the build process.
* If you encounter issues with tests, verify that your file paths (such as `data.txt`) are correctly set.
