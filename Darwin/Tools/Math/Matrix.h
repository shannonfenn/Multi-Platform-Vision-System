#pragma once

#include <vector>
#include <math.h>
#include <iomanip>
#include "Vector2.h"
#include "Vector3.h"

using std::ostream;

class Matrix
{
private:
	int		M;			// number of rows
	int		N;			// number of columns
	double*	X;			// matrix pointer
public:
	int		getm()	const;		// return number  of rows
	int		getn()	const;		// return number  of columns
	double*	getx()	const;		// return pointer to array	 

	// Constructors
	Matrix();
	Matrix(int m, int n, bool I=false);
	~Matrix();
	Matrix(const Matrix& a);

    Matrix(const Vector2<float>& vector);
    Matrix(const Vector2<double>& vector);
    Matrix(const Vector3<float>& vector);
    Matrix(const Vector3<double>& vector);

	
    Matrix transp() const; // Matrix Transpose
    Matrix getRow(int index) const; // Get Row
    Matrix getCol(int index) const; // Get Column
	void setRow(int index, Matrix in); // Set Row
	void setCol(int index, Matrix in); // Set Column

    void swapRows(unsigned int index1, unsigned int index2);
	Matrix&	operator =  (const Matrix& a); // Overloaded Operator
    inline double* operator []	(int i)	const {return &X[i*N];};
    inline double& operator() (int i, int j){return X[i*N+j];};
	
	void print(); // print values
    std::vector<float> asVector();
    bool isValid() const;
    bool operator ==(const Matrix& b) const;
    bool operator !=(const Matrix& b) const {return (!((*this) == b));}
};

// Overloaded Operators
Matrix	operator +  (const Matrix& a, const Matrix& b);
Matrix	operator -  (const Matrix& a, const Matrix& b);
Matrix	operator -  (const Matrix& a, const double& b);
Matrix	operator *  (const Matrix& a, const Matrix& b);
Matrix	operator *  (const double& a, const Matrix& b);
Matrix	operator *  (const Matrix& a, const double& b);
Matrix	operator /  (const Matrix& a, const double& b);

// Create 4x4 matrix from float vector.
Matrix Matrix4x4fromVector(const std::vector<float>& source);

// 2x2 Matrix Inversion
Matrix Invert22(const Matrix& a);
// concatenation
Matrix horzcat(Matrix a, Matrix b);
Matrix vertcat(Matrix a, Matrix b);
Matrix diagcat(Matrix a, Matrix b);
Matrix cholesky(Matrix P);
Matrix HT(Matrix A);
Matrix QR_Householder(const Matrix& A);
Matrix diag(const Matrix& A);

inline double convDble(const Matrix& a) { return a[0][0]; } // Convert 1x1 matrix to Double

inline	 int		Matrix::getm() const { return M; }
inline	 int		Matrix::getn() const { return N; }
inline	 double*	Matrix::getx() const { return X; }

double determinant(const Matrix& mat);
Matrix CofactorMatrix(const Matrix& mat);
Matrix InverseMatrix(const Matrix& mat);

Matrix CramersRuleInverse(const Matrix& mat);
Matrix GaussJordanInverse(const Matrix& mat);

/*!
  @brief Performs an update on the cholesky composition. If S is the original Cholesky fac-
    tor of P = AA^T, then the Cholesky factor of the rank-1 update (or downdate) cholupdate
    P = P +- sqrt(v) *UU^T, where a positive v performs an update, while a negative v performs a downdate.
    @param S The original Cholesky decomposition.
    @param U The update factor as shown above.
    @param v The signed update factor as shown above.
  */

Matrix CholeskyUpdate(Matrix S, Matrix U, float v);

std::ostream& operator <<(std::ostream& out, const Matrix &mat);

double dot(const Matrix& mat1, const Matrix& mat2);

void WriteMatrix(std::ostream& out, const Matrix &mat);
Matrix ReadMatrix(std::istream& in);

Matrix xRotMatrix(double angle);
Matrix yRotMatrix(double angle);
Matrix zRotMatrix(double angle);


