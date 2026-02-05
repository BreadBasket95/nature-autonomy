/**
 * \class Matrix
 * 
 * A matrix class that supplies what glm was lacking, namely arbitrarily sized
 * matrices with operations such as transpose and inverse.
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */
#ifndef AVT_341_MATRIX_H
#define AVT_341_MATRIX_H

#include <vector>
#include <iostream>

namespace nature{
namespace math{

class Matrix {
 public: 
  /**
   * @brief Construct an empty matrix.
   * @details Initializes a 0x0 matrix with no elements.
   */
  Matrix();
  
  /**
   * @brief Construct a matrix with specified dimensions.
   * @param nrows Number of rows.
   * @param ncols Number of columns.
   * @details Allocates and zero-initializes storage.
   */
  Matrix(int nrows, int ncols);

  /**
   * @brief Destroy the matrix and release storage.
   * @details Uses default cleanup.
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
   * @param i Row index.
   * @param j Column index.
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
   * @details Used in inversion and singularity checks.
   */
  double Determinant();
 
  /**
   * @brief Resize the matrix and clear elements to zero.
   * @param nrows New number of rows.
   * @param ncols New number of columns.
   * @details Reallocates storage and zeroes all elements.
   */
  void Resize(int nrows, int ncols);

  /**
   * @brief Get the number of columns.
   * @return Column count.
   * @details Used for dimension checks.
   */
  int GetNumCols()const {return ncols_;}

  /**
   * @brief Get the number of rows.
   * @return Row count.
   * @details Used for dimension checks.
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
   * @details Debug helper used during coordinate conversion development.
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

} //namespace math
} //namespace nature

#endif
