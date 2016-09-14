#pragma once

#include <boost/numeric/ublas/lu.hpp>

template<class matrix_T>
double GetDeterminant(boost::numeric::ublas::matrix_expression<matrix_T> const& mat_r)
{
	double det = 1.0;

	matrix_T mLu(mat_r());
	boost::numeric::ublas::permutation_matrix<std::size_t> pivots(mat_r().size1());

	int is_singular = boost::numeric::ublas::lu_factorize(mLu, pivots);

	if (!is_singular)
	{
		for (std::size_t i=0; i < pivots.size(); ++i)
		{
			if (pivots(i) != i)
				det *= -1.0;

			det *= mLu(i,i);
		}
	}
	else
		det = 0.0;

	return det;
}

template<class matrix_T>
double GetTrace(boost::numeric::ublas::matrix_expression<matrix_T> const& mat_r) {
	double trace = 0.0;

	int iDim1 = mat_r().size1();
	int iDim2 = mat_r().size2();

	if (iDim1==iDim2) {
		matrix_T A(mat_r());
		for (int i=0; i < iDim1; ++i) {
			trace += A(i,i);
		}
	}
	
	return trace;
}

template<class M1, class M2>
bool InvertMatrix (const M1& input, M2& inv) {
	using namespace boost::numeric::ublas;
	using boost::numeric::ublas::permutation_matrix;
	//typedef permutation_matrix<std::size_t> pmatrix;
	// create a working copy of the input
	M1 A(input);
	// create a permutation matrix for the LU-factorization
	permutation_matrix<std::size_t> pm(A.size1());
	// perform LU-factorization
	int res = lu_factorize(A,pm);
	if( res != 0 ) return false;

	// create identity matrix of "inverse"
	inv.assign(identity_matrix<typename M1::value_type>(A.size1()));

	// backsubstitute to get the inverse
	lu_substitute(A, pm, inv);

	return true;
}
