# Lidar Processing and 3D Mesh Generation

This project is the workflow I use to create three-dimensional building and terrain models from LIDAR `.las` files. The workflow relies on a number of tools and libraries including libLAS and CGAL.

## License

Please review the licenses of the included libraries.

## Prequisites and Dependencies

This guide was tested and developed on Ubuntu 13.04 64-bit.

### Boost C++ Libraries

@TODO

```bash
# Install Boost libraries
sudo tar -xvzf boost_1_53_0.tar.gz -C /opt
cd /opt/boost_1_53_0/
sudo ./bootstrap.sh
sudo ./b2
```

### CMake

@TODO

```bash
sudo tar -xvzf cmake-2.8.11.1.tar.gz -C /opt
cd /opt/cmake-2.8.11.1/
sudo ./bootstrap
sudo make && sudo make install
```

### libLAS

The libLAS library provides utilities to manipulate `.las` files. Specific to our workflow, this library allows us to

```bash
sudo mkdir makefiles
cd makefiles/
sudo cmake -G "Unix Makefiles" -DBOOST_ROOT=/opt/boost_1_53_0 ../
sudo make
sudo make install
```

### PCL

The Point Cloud Library

### Boost Libraries

*Installed in prior step.*

#### FLANN



#### 

#### Visualization Toolkit

### CGAL

http://www.cgal.org/Manual/latest/doc_html/installation_manual/Chapter_installation_manual.html

The Computational Geometry Algorithms Library (CGAL) provides a C++ API with optimized and best-practive algorithms. This library will provide the utilities for point set processing and 3D mesh generation.

#### GMP

```bash
# TODO: tar.lz
/opt/gmp-5.1.2
```

#### zlib

#### Qt

http://download.qt-project.org/official_releases/qt/5.0/5.0.2/single/qt-everywhere-opensource-src-5.0.2.tar.gz

http://stackoverflow.com/questions/11662529/building-qt-libraries-on-ubuntu-linux

#### CGAL SWIG Bindings

For various reasons I chose to build and install the CGAL SWIG bindings so that I had the option to program with Python.

## Point Set Processing

### LAS Manipulation 

### File Conversion

### Analysis

### Outlier Removal

### Simplication

### Normal Estimation

### Normal Orientation

## 3D Polyhedron Surface

## 3D Mesh Construction

Meshing domain is the 3D polyhedron created in the previous section, and it has sharp features and variable sizing field.

http://cgal-discuss.949826.n4.nabble.com/Export-mesh-td950677.html