/*
 * yaml_eig_cov.h
 *
 *  Created on: Aug 27, 2015
 *      Author: subhransu
 */

#ifndef YAML_EIG_COV_H
#define YAML_EIG_COV_H
#include<yaml-cpp/yaml.h> //using yaml version > 0.5
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

namespace YAML
{
template<typename T>
struct convert< pair<string,T> >
{
  static Node encode(const pair<string,T>& string_mat) {
    Node node;
    node.push_back(string_mat.first);
    T& mat = string_mat.second;
    for (int i = 0; i < mat.rows(); i++)
      for(int j=0; j<mat.cols();j++)
        node.push_back(mat(i,j));
    return node;
  }

  static bool decode(const Node& node, pair<string,T>& string_mat)
  {

    if(!node.IsSequence() )
      return false;
    else if(node.size()==1)
    {
      string_mat.first = node[0].as<string>();
      string_mat.second.setZero();
      return true;
    }
    else if(node.size()-1 != string_mat.second.size())
      return false;

    string_mat.first = node[0].as<string>();
    T& mat = string_mat.second;
    for (int i = 0; i < mat.rows(); i++)
    {
      for(int j = 0; j < mat.cols(); j++)
      {
        int k = j+ i*mat.cols();
        mat(i,j) =  node[k+1].as<double>();
      }
    }
    return true;
  }
};


//for Matrix<T,dynamic,1> so basically VectorXd, VectorXi ...
template<typename T>
struct convert<Matrix<T,Dynamic,1>>
{
  static Node encode(const VectorXd& vec)
  {
    Node node;
    for (int32_t i = 0; i < vec.size(); i++)
      node.push_back(vec(0));
    return node;
  }

  static bool decode(const Node& node, VectorXd& vec)
  {
    vec.resize(node.size());
    for (int32_t i = 0; i < node.size(); i++)
        vec[i] =  node[i].as<T>();
    return true;
  }
};

//for Matrix<double,n,m>
template<typename T, int m, int n>
struct convert<Matrix<T,m,n>>
{
  static Node encode(const Matrix<T,m,n>& mat)
  {
    Node node;
    for (int i = 0; i < mat.rows(); i++)
      for(int j=0; j<mat.cols();j++)
        node.push_back(mat(i,j));
    return node;

  }

  static bool decode(const Node& node, Matrix<T,m,n>& mat)
  {

    if(!node.IsSequence() )
      return false;
    else if(node.size()!= mat.size())
      return false;

    for (int i = 0; i < mat.rows(); i++)
    {
      for(int j = 0; j < mat.cols(); j++)
      {
        int k = j+ i*mat.cols();
        mat(i,j) =  node[k].as<T>();
      }
    }
    return true;
  }
};

//for Matrix<double,n,dynamic>
template<typename T, int m>
struct convert<Matrix<T,m,Dynamic>>
{
  static Node encode(const Matrix<T,m,Dynamic>& mat)
  {
    Node node;
    for (int i = 0; i < mat.rows(); i++)
      for(int j=0; j<mat.cols();j++)
        node.push_back(mat(i,j));
    return node;

  }

  static bool decode(const Node& node, Matrix<T,m,Dynamic>& mat)
  {

    if(!node.IsSequence() )
      return false;
    else if(node.size()%mat.RowsAtCompileTime!=0)
      return false;

    int cols = node.size()/mat.RowsAtCompileTime;
    mat.resize(mat.RowsAtCompileTime,cols);
    for (int i = 0; i < mat.rows(); i++)
    {
      for(int j = 0; j < mat.cols(); j++)
      {
        int k = j+ i*mat.cols();
        mat(i,j) =  node[k].as<T>();
      }
    }
    return true;
  }
};

//for Matrix<double,Dynamic,n>
template<typename T, int n>
struct convert< Matrix<T,Dynamic,n> >
{
  static Node encode(const Matrix<T,Dynamic,n>& mat)
  {
    Node node;
    for (int i = 0; i < mat.rows(); i++)
      for(int j=0; j<mat.cols();j++)
        node.push_back(mat(i,j));
    return node;

  }

  static bool decode(const Node& node, Matrix<T,Dynamic,n>& mat)
  {

    if(!node.IsSequence() )
      return false;
    else if(node.size()%mat.ColsAtCompileTime!=0)
      return false;

    int rows = node.size()/mat.ColsAtCompileTime;
    mat.resize(rows,mat.ColsAtCompileTime);
    for (int i = 0; i < mat.rows(); i++)
    {
      for(int j = 0; j < mat.cols(); j++)
      {
        int k = j+ i*mat.cols();
        mat(i,j) =  node[k].as<T>();
      }
    }
    return true;
  }
};

}


#endif /* YAML_EIG_COV_H */
