/*
Non-Commercial License - Mississippi State University Off-Road Traversability Algorithm

REPO: https://gitlab.com/cgoodin/off_road_traversability

CONTACT: cgoodin@cavs.msstate.edu

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

CITATION:
Goodin, C., Dabbiru, L., Hudson, C., Mason, G., Carruth, D., & Doude, M. (2021, April).
Fast terrain traversability estimation with terrestrial lidar in off-road autonomous navigation.
In Unmanned Systems Technology XXIII (Vol. 11758, p. 117580O). International Society for Optics and Photonics.

NOTICE:
Do not share or distribute. Software is authorized for use only by the approved recepient.

Copyright 2022 (C) Mississippi State University
*/
#ifndef TRAVLIB_MATRIX_H
#define TRAVLIB_MATRIX_H

#include <vector>
#include <iostream>

namespace traverselib{

class Matrix {
 public: 
  /**
   * @brief Construct an empty matrix.
   * @details Initializes a 0x0 matrix with no elements. Used as a default
   *          placeholder when building intermediate results.
   */
  Matrix();
  
  /**
   * @brief Construct a matrix with specified dimensions.
   * @param nrows Number of rows.
   * @param ncols Number of columns.
   * @details Allocates and zero-initializes the element storage.
   */
  Matrix(int nrows, int ncols);

  /**
   * @brief Destroy the matrix and release storage.
   * @details Uses default cleanup; included for explicit ownership semantics.
   */
  ~Matrix();

  /**
   * @brief Copy-construct a matrix.
   * @param m Matrix to copy.
   * @details Duplicates dimensions and element storage.
   */
  Matrix (const Matrix &m);
  
  /**
   * @brief Return the transpose of the matrix.
   * @return New matrix with rows and columns swapped.
   * @details Allocates a new matrix and copies elements into transposed positions.
   */
  Matrix Transpose();

  /**
   * @brief Return the minor of element (i,j).
   * @param i Row index of the element.
   * @param j Column index of the element.
   * @return Minor matrix with row i and column j removed.
   * @details Used in determinant and inverse calculations.
   */
  Matrix GetMinor(int i, int j);

  /**
   * @brief Compute the inverse of the matrix.
   * @return Inverse matrix, or a zero matrix if the inverse does not exist.
   * @details Uses determinant/minors; returns a zero matrix on singular input.
   */
  Matrix Inverse();

  /**
   * @brief Compute the determinant of the matrix.
   * @return Determinant value.
   * @details Uses recursive expansion; used by Inverse and FitToPoints.
   */
  double Determinant();
 
  /**
   * @brief Resize the matrix and clear elements to zero.
   * @param nrows New number of rows.
   * @param ncols New number of columns.
   * @details Reallocates storage and zeros all elements.
   */
  void Resize(int nrows, int ncols);

  /**
   * @brief Get the number of columns.
   * @return Column count.
   * @details Used for dimension checking in operators.
   */
  int GetNumCols()const {return ncols_;}

  /**
   * @brief Get the number of rows.
   * @return Row count.
   * @details Used for dimension checking in operators.
   */
  int GetNumRows()const {return nrows_;}

  /**
   * @brief Access an element by (row, column).
   * @param i Row index.
   * @param j Column index.
   * @return Element value.
   * @details Uses the internal flat index mapping.
   */
  double GetElement(int i, int j)const {return elements_[GetIndex(i,j)];}

  /**
   * @brief Print the matrix to stdout.
   * @details Debug helper used in development and testing.
   */
  void Print();

  /**
   * @brief Mutable element access with parentheses.
   * @param i Row index.
   * @param j Column index.
   * @return Reference to the element.
   * @details Enables matrix-style indexing in algorithms.
   */
  double& operator() (int i, int j){
    return elements_[GetIndex(i,j)];
  }
  /**
   * @brief Const element access with parentheses.
   * @param i Row index.
   * @param j Column index.
   * @return Value of the element.
   * @details Enables matrix-style indexing in const contexts.
   */
  double operator() (int i, int j) const {
    return elements_[GetIndex(i,j)];
  }

  /**
   * @brief Compare matrix dimensions.
   * @param a Matrix to compare against.
   * @return True if dimensions match; false otherwise.
   * @details Used by operator overloads to guard invalid math.
   */
  bool CompareSize(const Matrix& a)const;

 private:
  std::vector<double> elements_;
  /**
   * @brief Convert 2D indices to a flat array index.
   * @param i Row index.
   * @param j Column index.
   * @return Linear index into elements_.
   * @details Uses row-major layout.
   */
  int GetIndex(int i, int j) const;
  
  /**
   * @brief Compute the sign for a cofactor term.
   * @param i Row index.
   * @return +1 or -1 depending on parity.
   * @details Used in determinant expansion.
   */
  int PermuteSign(int i);
  int nrows_;
  int ncols_;
};

//----- Matrix operators --------------------------------------//
/**
 * @brief Add two matrices.
 * @param a Left-hand matrix.
 * @param b Right-hand matrix.
 * @return Sum matrix if dimensions match; otherwise zeros with warning.
 * @details Performs element-wise addition.
 */
inline Matrix operator+(const Matrix& a, const Matrix& b) {
  Matrix c(a.GetNumRows(), a.GetNumCols());
  if (a.CompareSize(b)){
    for (int i = 0; i<a.GetNumRows(); i++){
      for (int j = 0; j<a.GetNumCols(); j++){
	c(i,j) = a(i,j) + b(i,j);
      }
    }
  }
  else {
    std::cerr<<"Warning, attempted to add "<<a.GetNumRows()<<"x"<<
      a.GetNumCols()<<" matrix and "<<b.GetNumRows()<<"x"<<b.GetNumCols()
	     <<"matrix."<<std::endl;
  }
  return c; 
}

/**
 * @brief Subtract two matrices.
 * @param a Left-hand matrix.
 * @param b Right-hand matrix.
 * @return Difference matrix if dimensions match; otherwise zeros with warning.
 * @details Performs element-wise subtraction.
 */
inline Matrix operator-(const Matrix& a, const Matrix& b) {
  Matrix c(a.GetNumRows(), a.GetNumCols());
  if (a.CompareSize(b)){
    for (int i = 0; i<a.GetNumRows(); i++){
      for (int j = 0; j<a.GetNumCols(); j++){
	c(i,j) = a(i,j) - b(i,j);
      }
    }
  }
  else {
    std::cerr<<"Warning, attempted to subtract "<<a.GetNumRows()<<"x"<<
      a.GetNumCols()<<" matrix and "<<b.GetNumRows()<<"x"<<b.GetNumCols()
	     <<"matrix."<<std::endl;
  }
  return c; 
}

/**
 * @brief Multiply a matrix by a scalar.
 * @param m Matrix to scale.
 * @param s Scalar multiplier.
 * @return Scaled matrix.
 * @details Multiplies each element by s.
 */
inline Matrix operator*(const Matrix& m, const double s) {
  Matrix b(m.GetNumRows(), m.GetNumCols());
  for (int i = 0; i<m.GetNumRows(); i++){
    for (int j = 0; j<m.GetNumCols(); j++){
      b(i,j) = s*m(i,j);
    }
  }
  return b; 
}

/**
 * @brief Multiply a scalar by a matrix.
 * @param s Scalar multiplier.
 * @param m Matrix to scale.
 * @return Scaled matrix.
 * @details Delegates to matrix * scalar operator.
 */
inline Matrix operator*(const double s, const Matrix& m){
  return m*s; 
}

/**
 * @brief Divide a matrix by a scalar.
 * @param m Matrix to scale.
 * @param s Scalar divisor.
 * @return Scaled matrix.
 * @details Divides each element by s; caller must avoid s==0.
 */
inline Matrix operator/(const Matrix& m, const double s) {
  Matrix b(m.GetNumRows(), m.GetNumCols());
  for (int i = 0; i<m.GetNumRows(); i++){
    for (int j = 0; j<m.GetNumCols(); j++){
      b(i,j) = m(i,j)/s;
    }
  }
  return b; 
}

/**
 * @brief Multiply two matrices.
 * @param a Left-hand matrix.
 * @param b Right-hand matrix.
 * @return Product matrix if dimensions match; otherwise zeros with warning.
 * @details Performs standard matrix multiplication.
 */
inline Matrix operator*(const Matrix& a, const Matrix& b) {
  Matrix c(a.GetNumRows(), b.GetNumCols());
  if (a.GetNumCols()==b.GetNumRows()){
    for (int i = 0; i<a.GetNumRows(); i++){
      for (int j = 0; j<b.GetNumCols(); j++){
	for (int k=0; k<a.GetNumCols();k++){
	  c(i,j) = c(i,j) + a(i,k)*b(k,j);
	}
      }
    }
  }
  else{
    std::cerr<<"Warning: Attempted to multiply matrix with inner dimensions "
	     <<a.GetNumCols()<<" and "<<b.GetNumRows()<<std::endl;
  }   
  return c; 
}
//--- Done with operators-----------------------------------------//

} //namespace traverslib

#endif
