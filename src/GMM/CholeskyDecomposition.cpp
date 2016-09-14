#include "mapping_localization_shared/CholeskyDecomposition.h"

#include <boost/numeric/conversion/cast.hpp>

#undef NDEBUG
#include <cassert>

CholeskyDecomposition::CholeskyDecomposition(const mDouble& A) : L(A), isSpd(true)
{
    unsigned int i, j, n = A.size1();
    int k;
    double sum;
    if (L.size2() != n)
        throw std::runtime_error("CholeskyDecomposition: need square matrix");
    for (i = 0; i < n; i++) {
        for (j = i; j < n; j++) {
            for (sum = L(i,j), k = int(i - 1); k >= 0; k--)
                sum -= L(i,unsigned(k)) * L(j,unsigned(k));
            if (i == j) {
                isSpd &= sum > 0.0;
                //if (sum <= 0.0)
                //	throw("Cholesky failed");
                L(j,i) = sqrt(sum);
            } else
                L(j,i) = sum / L(i,i);
        }
    }
    for (i = 0; i < n; i++)
        for (j = 0; j < i; j++)
            L(j,i) = 0.;
}

const CholeskyDecomposition::mDouble& CholeskyDecomposition::getL() const
{
    return L;
}

bool CholeskyDecomposition::isSPD() const
{
    return isSpd;
}

// Computes the determinant by squaring the product of the diagonals of L.
// Returns 0 if the matrix A is not SPD
double CholeskyDecomposition::det() const
{
    if (!isSpd)
        return 0;

    double d = 1.0;
    for (unsigned i = 0; i < L.size1(); i++)
        d *= L(i,i);

    return d*d;
}

double CholeskyDecomposition::logdet() const {
    double sum = 0.;
    for (unsigned i = 0; i < L.size1(); i++)
        sum += log(L(i, i));
    return 2. * sum;
}

CholeskyDecomposition::mDouble CholeskyDecomposition::solve(const mDouble& B) const
{
    unsigned int m = L.size1();
    assert(B.size1() == m);
    assert(isSpd);

    // Solve L*Y = B
    mDouble YT(B.size2(), B.size1(), 0.0);

    // most of the time, B will have one column.
    for (unsigned cidx = 0; cidx < B.size2(); cidx++) {

        for (unsigned ridx = 0; ridx < B.size1(); ridx++) {
            using boost::numeric::ublas::row;
            using boost::numeric::ublas::inner_prod;
            double dot = inner_prod(row(L,ridx), row(YT,cidx));
            double err = (B(ridx,cidx) - dot) / L(ridx,ridx);
            YT(cidx,ridx) = err;
        }
    }

    using boost::numeric::ublas::trans;

    mDouble XT(B.size2(), B.size1(), 0.0);
    auto LT = trans(L);

    assert(B.size1() > 0);
    auto r0 = boost::numeric_cast<int>(B.size1()-1);

    // most of the time, Y will have one column.
    // Solve L'*X = Y
    for (unsigned cidx = 0; cidx < B.size2(); cidx++) {

        for (int ridx = r0; ridx >= 0; ridx--) {
            double dot = inner_prod(row(LT,unsigned(ridx)), row(XT,cidx));
            double err = (YT(cidx,unsigned(ridx)) - dot) / L(unsigned(ridx),unsigned(ridx));

            XT(cidx,unsigned(ridx)) = err;
        }
    }

    return trans(XT);
}

CholeskyDecomposition::vDouble CholeskyDecomposition::solve(const vDouble& b) const
{
    unsigned int m = L.size1();
    assert(b.size() == m && m > 0);
    assert(isSpd);

    // Solve L*Y = B
    vDouble YT(m, 0.0);

    for (unsigned ridx = 0; ridx < m; ridx++) {
        using boost::numeric::ublas::row;
        using boost::numeric::ublas::inner_prod;
        double dot = inner_prod(row(L,ridx), YT);
        double err = (b(ridx) - dot) / L(ridx,ridx);
        YT(ridx) = err;
    }

    using boost::numeric::ublas::trans;

    vDouble XT(m, 0.0);
    auto LT = trans(L);

    // most of the time, Y will have one column.
    // Solve L'*X = Y

    for (int ridx = boost::numeric_cast<int>(m-1); ridx >= 0; ridx--) {
        double dot = inner_prod(row(LT,unsigned(ridx)), XT);
        double err = (YT(unsigned(ridx)) - dot) / L(unsigned(ridx),unsigned(ridx));

        XT(unsigned(ridx)) = err;
    }

    return XT;
}

void CholeskyDecomposition::elsolve(const vDouble& b, vDouble& y) const {
    unsigned int i, j;
    unsigned int n = L.size1();
    double sum;
    if (b.size() != n || y.size() != n)
        throw std::runtime_error("CholeskyDecomposition::elsolve: bad lengths");
    for (i = 0; i < n; i++) {
        for (sum=b(i), j = 0; j < i; j++)
            sum -= L(i,j) * y(j);
        y(i) = sum / L(i,i);
    }
}

void CholeskyDecomposition::elmult(const vDouble& y, vDouble& b) const {
    unsigned int i, j;
    unsigned int n = L.size1();

    if (b.size() != n || y.size() != n)
        throw std::runtime_error("CholeskyDecomposition::elmult: bad lengths");
    for (i = 0; i < n; i++) {
        b[i] = 0;
        // actually "j <= i" and no line "b[i] += L(i,i) * y(i);", but gcc "cannot optimize possibly infinite loops":
        // if n == UINT_MAX, then j would loop to UINT_MAX (condition is true), and j++ would overflow to 0 (condition is true)
        for (j = 0; j < i; j++)
            b[i] += L(i,j) * y(j);
        b[i] += L(i,i) * y(i);
    }
}
