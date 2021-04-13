#include <iostream>
#include<vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 

using namespace std;
using Eigen::MatrixXd;


// return the adjacency matrix of a graph
// TODO: complete the graph class implementation or use a library like nGraph
//
Eigen::MatrixXd getAdjacencyMatrix();

/// compute eigenvalues of the Laplacian
//  A - adjacency matrix, D - degree matrix
//  return a colum vector of eigenvalues(D-A)
Eigen::MatrixXd getLaplacianEigenvalues(Eigen::MatrixXd, Eigen::MatrixXd);

/// get the top-k eigen values of each matrix
//  that adds upto a given energy value
Eigen::MatrixXd getTopKEigenvalues(MatrixXd, MatrixXd, float);

/// compute similarity of the 2 graphs
// topkmin [kx2]a
float similarity(Eigen::MatrixXd topkmin);


