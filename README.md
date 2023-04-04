# g2o_study
This repo includes several different versions of g2o, and some codes.  


## original g2o
Creating solver using newest g2o is as follow:
```cpp
g2o::SparseOptimizer optimizer;
optimizer.setVerbose(false);
g2o::OptimizationAlgorithmProperty solverProperty;
optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense",solverProperty));
```
Under this repo, only a demo source code is provided. Not executable.  
- C++ version: c++17. ("view_string.h" is needed, which is in c/7 by default.)
Check [RainerKuemmerle's g2o](https://github.com/RainerKuemmerle/g2o).

## orbslam3_g2o
This g2o version is from ORBSLAM3's Thirdparty library (modified, not a full-version of g2o), which is an "OLD" version.   
Creating solver using this g2o is as follow:
```cpp
typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
BlockSolverType::LinearSolverType *linearSolver = new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>();
BlockSolverType *block_sover = new BlockSolverType(linearSolver);
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(block_sover);
g2o::SparseOptimizer optimizer;
optimizer.setAlgorithm(solver);
```
Codes is from gaoxiang's [slambook1's codes](https://github.com/gaoxiang12/slambook/blob/master/ch6/g2o_curve_fitting/main.cpp), and also used in electron6's book ch4.

This code is executable:
- Compile the g2o under `orbslam3_g2o/Thirdparty/g2o`, which generate `libg2o` under 'lib' folder.
- Compile the `curve_fitting.cpp`, in which CMakeLists.txt, the g2o's lib (and directories) is assigned to the above folder.
- C++ version: c++14


## slambook2_g2o
This code is copied from gaoxiang's slambook2 ch6, which is a "NEWER" version, using intelligent pointer.  
Creating solver using this g2o is as follow:
```cpp
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);
```
This code is also executable:
- Compile the g2o under `slambook2_g2o/g2o`, and install the libs to system's default path.
- Compile the `curve_fitting.cpp`, in which CMakeLists.txt, the g2o's lib folder is the system default path.
- C++ version: c++14
