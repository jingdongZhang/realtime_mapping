#pragma once

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
/** Cholesky Decomposition.
    <P>
    For a symmetric, positive definite matrix A, the Cholesky decomposition
    is an lower triangular matrix L so that A = L*L'.
    <P>
    If the matrix is not symmetric or positive definite, the constructor
    returns a partial decomposition and sets an internal flag that may
    be queried by the isSPD() method.
*/
class CholeskyDecomposition
{
public:
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;
    CholeskyDecomposition(const mDouble& A);

    const mDouble& getL() const;
    bool isSPD() const;

    // Computes the determinant by squaring the product of the diagonals of L.
    // Returns 0 if the matrix A is not SPD
    double det() const;

    double logdet() const;

    mDouble solve(const mDouble& B) const;
    vDouble solve(const vDouble& b) const;

    void elsolve(const vDouble& b, vDouble& y) const;
    void elmult(const vDouble& y, vDouble& b) const;
private:
    mDouble L;

    bool isSpd;
};

template<unsigned N>
class c_CholeskyDecomposition
{
public:
    typedef boost::numeric::ublas::c_matrix<double, N, N> mat;
    typedef boost::numeric::ublas::c_vector<double, N> vec;
    typedef boost::numeric::ublas::matrix<double> mDouble;
    c_CholeskyDecomposition(const mat& A) : L(A), isSpd(true)
    {
        unsigned int i, j;
        int k;
        vec tmp;
        double sum;
        for (i = 0; i < N; i++) {
            for (j = i; j < N; j++) {
                for (sum = L(i,j), k = i - 1; k >= 0; k--)
                    sum -= L(i,k) * L(j,k);
                if (i == j) {
                    isSpd &= sum > 0.0;
                    //if (sum <= 0.0)
                    //  throw("Cholesky failed");
                    L(j,i) = sqrt(sum);
                } else
                    L(j,i) = sum / L(i,i);
            }
        }
        for (i = 0; i < N; i++)
            for (j = 0; j < i; j++)
                L(j,i) = 0.;
    }

    const mat& getL() const
    {
        return L;
    }

    bool isSPD() const
    {
        return isSpd;
    }

    // Computes the determinant by squaring the product of the diagonals of L.
    // Returns 0 if the matrix A is not SPD
    double det() const
    {
        if (!isSpd)
            return 0;

        double det = 1.0;
        for (unsigned i = 0; i < N; i++)
            det *= L(i,i);

        return det*det;
    }

    double logdet() const {
        double sum = 0.;
        for (unsigned i = 0; i < N; i++)
            sum += log(L(i, i));
        return 2. * sum;
    }

    mDouble solve(const mDouble& B) const
    {
        assert(B.size1() == N);
        assert(isSpd);

        // Solve L*Y = B
        mDouble YT(B.size2(), N, 0.0);

        // most of the time, B will have one column.
        for (unsigned cidx = 0; cidx < B.size2(); cidx++) {

            for (unsigned ridx = 0; ridx < N; ridx++) {
                using boost::numeric::ublas::row;
                using boost::numeric::ublas::inner_prod;
                double dot = inner_prod(row(L,ridx), row(YT,cidx));
                double err = (B(ridx,cidx) - dot) / L(ridx,ridx);
                YT(cidx,ridx) = err;
            }
        }

        using boost::numeric::ublas::trans;

        mDouble XT(B.size2(), N, 0.0);
        auto LT = trans(L);

        // most of the time, Y will have one column.
        // Solve L'*X = Y
        for (unsigned cidx = 0; cidx < B.size2(); cidx++) {

            for (int ridx = N-1; ridx >= 0; ridx--) {
                double dot = inner_prod(row(LT,ridx), row(XT,cidx));
                double err = (YT(cidx,ridx) - dot) / L(ridx,ridx);

                XT(cidx,ridx) = err;
            }
        }

        return trans(XT);
    }

    vec solve(const vec& b) const
    {
        assert(isSpd);

        // Solve L*Y = B
        vec YT = boost::numeric::ublas::zero_vector<double>(N);

        for (unsigned ridx = 0; ridx < N; ridx++) {
            using boost::numeric::ublas::row;
            using boost::numeric::ublas::inner_prod;
            double dot = inner_prod(row(L,ridx), YT);
            double err = (b(ridx) - dot) / L(ridx,ridx);
            YT(ridx) = err;
        }

        using boost::numeric::ublas::trans;

        vec XT = boost::numeric::ublas::zero_vector<double>(N);
        auto LT = trans(L);

        // most of the time, Y will have one column.
        // Solve L'*X = Y

        for (int ridx = N-1; ridx >= 0; ridx--) {
            double dot = inner_prod(row(LT,ridx), XT);
            double err = (YT(ridx) - dot) / L(ridx,ridx);

            XT(ridx) = err;
        }

        return XT;
    }

    void elsolve(const vec& b, vec& y) const {
        unsigned int i, j;
        double sum;
        for (i = 0; i < N; i++) {
            for (sum=b(i), j = 0; j < i; j++)
                sum -= L(i,j) * y(j);
            y(i) = sum / L(i,i);
        }
    }
    void elmult(const vec& y, vec& b) const {
        unsigned int i, j;
        for (i = 0; i < N; i++) {
            b[i] = 0;
            for (j = 0; j <= i; j++)
                b[i] += L(i,j) * y(j);
        }
    }
private:
    mat L;

    bool isSpd;
};
