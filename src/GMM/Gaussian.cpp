#include "Gaussian.h"

#undef NDEBUG
#include <cassert>

Gaussian::Gaussian()  : u(mDouble().size1(), 0.0), P(mDouble())
  , singular(false), n(mDouble().size2()), chol(mDouble())
{
}

/** @param P The covariance matrix. A mean of zero is assumed. **/
Gaussian::Gaussian(const mDouble& _P)
  : u(_P.size1(), 0), P(_P)
  , singular(false), n(P.size2()), chol(P)
{
    init();
}

/** @param P Covariance matrix
    @param u Mean vector
**/
Gaussian::Gaussian(const mDouble& _P, const vDouble& _u)
  : u(_u)
  , P(_P)
  , singular(false)
  , n(P.size2())
  , chol(P)
{
    init();
}

void Gaussian::init()
{
    Pdet = chol.det();
    if (Pdet==0) // if they pass in a zero covariance matrix, produce exact answers.
    {
        singular = true;
        Pscale = 0;
    }
    else
    {
        Pscale = 1.0/std::pow(2.0*M_PI, double(n)/2.0)/std::sqrt(Pdet);
    }

    logPscale = -0.5 * chol.logdet() - double(n) / 2.0 * std::log(2.0*M_PI);
}

///** Return an Nx1 vector drawn from the distribution **/
//CholeskyDecomposition::vDouble Gaussian::sample() const
//{
//    return sample(staticr);
//}

/** Return an Nx1 vector drawn from the distribution using the provided Random object **/
Gaussian::vDouble Gaussian::sample() const
{
    if (singular)
        return u;

    int n = P.size2();
    vDouble x(n);

    for (int i = 0; i <n; i++)
        x[i] = dist(generator);
    vDouble tmp(n);
    chol.elmult(x, tmp);
    return tmp + u;
}

/** compute probability of Nx1 vector v **/
double Gaussian::prob(const vDouble& v) const
{
    if (singular)
        throw std::runtime_error("Gaussian::prob: singular");

    return Pscale * unscaledProb(v);
}

/** compute probability of Nx1 vector v **/
double Gaussian::unscaledProb(const vDouble& v) const
{
    if (singular)
        throw std::runtime_error("Gaussian::unscaledProb: singular");

    vDouble w(n);
    for (int i = 0; i < n; i++)
        w(i) = v(i) - u(i);
    vDouble t(n);
    chol.elsolve(w, t);
    double e = 0.;
    for (int i = 0; i < n; i++)
        e += t(i) * t(i);
    double p = std::exp(-0.5 * e);

    return p;
}

/** compute log probability of Nx1 vector v **/
double Gaussian::logProb(const vDouble& v) const
{
    if (singular)
        return 0;

    // special case 3x3 for performance
    if (v.size() == 3)
    {
        const mDouble& L = chol.getL();

        double a = v[0] - u[0];
        double b = v[1] - u[1];
        double c = v[2] - u[2];

        double y0 = a / L(0,0);
        double y1 = (b - L(1,0) * y0) / L(1,1);
        double y2 = (c - L(2,0) * y0 - L(2,1) * y1) / L(2,2);

        return logPscale - 0.5 * (y0 * y0 + y1 * y1 + y2 * y2);
    }

    vDouble w(n);
    for (int i = 0; i < n; i++)
        w(i) = v(i) - u(i);
    vDouble t(n);
    chol.elsolve(w, t);
    double e = 0.;
    for (int i = 0; i < n; i++)
        e += t(i) * t(i);
    return logPscale - 0.5 * e;
}

/** compute log probability of 3x1 vector v **/
double Gaussian::logProb(const vDouble3& v) const
{
    if (singular)
        return 0;

    const mDouble& L = chol.getL();

    double a = v[0] - u[0];
    double b = v[1] - u[1];
    double c = v[2] - u[2];

    double y0 = a / L(0,0);
    double y1 = (b - L(1,0) * y0) / L(1,1);
    double y2 = (c - L(2,0) * y0 - L(2,1) * y1) / L(2,2);

    return logPscale - 0.5 * (y0 * y0 + y1 * y1 + y2 * y2);
}

/** Return inverse of the covariance matrix. **/
//mDouble Gaussian::getPinv()
//{
//	if (singular)
//		throw std::exception();
//    return Pinv;
//}

//double Gaussian::chi2(const vDouble& v) const
//{
//	if (singular)
//		throw("singular");
//    // special case 3x3 for performance
//    if (v.size() == 3)
//    {
//        double a = v[0] - u[0];
//        double b = v[1] - u[1];
//        double c = v[2] - u[2];
//
//        // assume symmetric Pinv (of course!)
//        double d = Pinv(0,0), e = Pinv(0,1), f = Pinv(0,2);
//        double g = Pinv(1,1), h = Pinv(1,2);
//        double i = Pinv(2,2);
//
//        return a*a*d + b*b*g + c*c*i + 2*(a*b*e + a*c*f + b*c*h);
//    }
//
//    vDouble w = LinAlg::subtract(v, u);
//    return LinAlg::dotProduct(w, LinAlg::times(Pinv,w));
//}

double Gaussian::getMahalanobisDistance(const vDouble& x) const
{
	if (singular)
		throw std::runtime_error("Gaussian::getMahalanobisDistance: singular");
    //vDouble dx = LinAlg::subtract(x, u);
    //return std::sqrt(LinAlg::dotProduct(dx, LinAlg::times(Pinv,dx)));

    vDouble w(n);
    for (int i = 0; i < n; i++)
        w(i) = x(i) - u(i);
    vDouble t(n);
    chol.elsolve(w, t);
    double e = 0.;
    for (int i = 0; i < n; i++)
        e += t(i) * t(i);
    return std::sqrt(e);
}

bool Gaussian::isValid() const {
    return P.size1() !=0 && P.size2() != 0;
}

double trace(const boost::numeric::ublas::matrix<double>& A) {
    assert(A.size1() == A.size2());
    double x = 0.0;
    for (unsigned i = 0; i < A.size1(); i++)
        x += A(i,i);
    return x;
}

double norm_kld(const Gaussian& N0, const Gaussian& N1) {
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;

    assert(N0.getDimension() == N1.getDimension());
    if(N0.isSingular() || N1.isSingular())
        return std::numeric_limits<double>::quiet_NaN();
    using namespace boost::numeric::ublas;
    const mDouble& sigma0 = N0.getCovariance(), sigma1 = N1.getCovariance();
    mDouble sigma1inv;
    auto good = InvertMatrix(sigma1, sigma1inv);
    assert(good);
    vDouble d = N1.getMean() - N0.getMean();
    unsigned k = N0.getDimension();
    return 0.5 * (trace(prod(sigma1inv, sigma0)) + inner_prod(d, prod(sigma1inv, d)) - k - log(N0.getDet() / N1.getDet()));
}

Gaussian productOf2Gaussians(const Gaussian& gaussianA, const Gaussian& gaussianB) {
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;

    assert(gaussianA.getDimension() == gaussianB.getDimension());
    const uint32_t n = gaussianA.getDimension();

    const vDouble& a = gaussianA.getMean();
    const vDouble& b = gaussianB.getMean();
    const mDouble& A = gaussianA.getCovariance();
    const mDouble& B = gaussianB.getCovariance();

    mDouble ApB = A + B;
    mDouble ApBInv(n, n);
    bool good = InvertMatrix(ApB, ApBInv);
    if (!good)
        return Gaussian();
    const vDouble& t1 = prod(ApBInv, a);
    const vDouble& t2 = prod(B, t1);
    const vDouble& t3 = prod(ApBInv, b);
    const vDouble& t4 = prod(A, t3);
    vDouble mean = t2 + t4;

    const mDouble& t5 = prod(ApBInv, B);
    mDouble cov = prod(A, t5);
    return Gaussian(cov, mean);
}
