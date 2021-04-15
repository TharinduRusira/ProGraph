#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <ngraph.hpp>

using namespace std;
//using Eigen::MatrixXd;
using namespace Eigen;
using namespace NGraph;

// return the adjacency matrix of a graph
Eigen::MatrixXi getAdjacencyMatrix(Graph);

// return the degree matrix of a graph
Eigen::MatrixXi getDegreeMatrix(Graph);

/// compute eigenvalues of the Laplacian
//  A - adjacency matrix, D - degree matrix
//  return a column vector of eigenvalues(D-A)
Eigen::MatrixXd getLaplacianEigenvalues(Eigen::MatrixXd, Eigen::MatrixXd);

/// get the top-k eigen values of each matrix
//  that adds upto a given energy value
Eigen::MatrixXd getTopKEigenvalues(MatrixXd, MatrixXd, float);

/// compute similarity of the 2 graphs
// topkmin [kx2]a
float similarity(Eigen::MatrixXd topkmin);


