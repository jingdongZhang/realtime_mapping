#pragma once

#include "CholeskyDecomposition.h"
#include "MatrixSolve.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <random>

/** Representation of an N-dimensional multi-gaussian. **/
class Gaussian
{
public:
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;
    typedef boost::numeric::ublas::c_matrix<double, 3, 3> mDouble3;
    typedef boost::numeric::ublas::c_vector<double, 3> vDouble3;
private:
    vDouble u;          // mean   
    mDouble P;          // The covariance matrix
    double  Pdet;
    double  Pscale;     // the coefficient out front.
    double  logPscale;  // log of Pscale
    bool    singular;
    int     n;          // number of variables
    mutable std::mt19937_64 generator;
    mutable std::normal_distribution<> dist;

    CholeskyDecomposition chol;

    void init();

public:
	Gaussian();

    /** @param P The covariance matrix. A mean of zero is assumed. **/
    explicit Gaussian(const mDouble& P);

    /** @param P Covariance matrix
        @param u Mean vector
    **/
    explicit Gaussian(const mDouble& P, const vDouble& u);

    /** Returns the number of variables described by this multi-gaussian. **/
    int getDimension() const
    {
        return P.size1();
    }

    /** Return an Nx1 vector drawn from the distribution **/
    vDouble sample() const;

    /** compute probability of Nx1 vector v **/
    double prob(const vDouble& v) const;
    double unscaledProb(const vDouble& v) const;

    /** compute log probability of Nx1 vector v **/
    double logProb(const vDouble& v) const;
    double logProb(const vDouble3& v) const;

    double getMahalanobisDistance(const vDouble& x) const;

    const mDouble& getCovariance() const           { return P; }
    const vDouble& getMean() const                 { return u; }
    bool isSingular() const                 { return singular; }
    double getDet() const                   { return Pdet; }
    bool isValid() const;
    const CholeskyDecomposition& getChol() const { return chol; }
};

template<unsigned N>
class c_Gaussian
{
public:
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;
    typedef boost::numeric::ublas::c_vector<double, N> vector_t;
    typedef boost::numeric::ublas::c_matrix<double, N, N> matrix_t;
private:
    vector_t u;          // mean
    matrix_t P;          // The covariance matrix
    double  Pdet;
    double  Pscale;     // the coefficient out front.
    double  logPscale;  // log of Pscale
    bool    singular;
    mutable std::mt19937_64 generator;
    mutable std::normal_distribution<> dist;

    c_CholeskyDecomposition<N> chol;

    void init();

public:
    c_Gaussian()  : u(boost::numeric::ublas::zero_vector<double>(N)), P(boost::numeric::ublas::zero_matrix<double>(N, N)), Pdet(0), Pscale(0), logPscale(0)
      , singular(true), chol(boost::numeric::ublas::zero_matrix<double>(N, N)) {}

    /** @param P The covariance matrix. A mean of zero is assumed. **/
    c_Gaussian(const matrix_t& _P)
      : u(boost::numeric::ublas::zero_vector<double>(N)), P(_P)
      , singular(false), chol(P)
    {

        init();
    }

    /** @param P Covariance matrix
        @param u Mean vector
    **/
    c_Gaussian(const matrix_t& _P, const vector_t& _u)
      : u(_u)
      , P(_P)
      , singular(false)
      , chol(P)
    {
        init();
    }

    /** Returns the number of variables described by this multi-gaussian. **/
    int getDimension() const
    {
        return P.size1();
    }

    /** Return an Nx1 vector drawn from the distribution **/
    template<class Generator>
    vector_t sample(Generator& g) const
    {
//        std::cout << "Gaussian::sample" << std::endl;
//        std::cout << "L: " << chol.getL() << std::endl;
        if (singular) {
//            std::cout << "singular" << std::endl;
            return u;
        }

        int n = P.size2();
        vector_t x(n);

        for (int i = 0; i <n; i++)
            x[i] = dist(g);
//        std::cout << "N(0,1)^3 sample: " << x << std::endl;
        vector_t tmp(n);
        chol.elmult(x, tmp);
//        std::cout << "scaled N(0,1)^3 sample: " << tmp << std::endl;
        return tmp + u;
    }

    vector_t sample() const
    {
        return sample(generator);
    }

    /** compute probability of Nx1 vector v **/
    double prob(const vector_t& v) const
    {
        if (singular)
            throw std::runtime_error("Gaussian::prob: singular");

        return Pscale * unscaledProb(v);
    }

    double unscaledProb(const vector_t& v) const;

    /** compute log probability of Nx1 vector v **/
    double logProb(const vector_t& v) const;

    double getMahalanobisDistance(const vector_t& x) const;

    const matrix_t& getCovariance() const           { return P; }
    const vector_t& getMean() const                 { return u; }
    bool isSingular() const                 { return singular; }
    double getDet() const                   { return Pdet; }
    bool isValid() const {
        return P.size1() !=0 && P.size2() != 0;
    }
    const c_CholeskyDecomposition<N>& getChol() const { return chol; }
};

template<unsigned N>
void c_Gaussian<N>::init()
{
    Pdet = chol.det();
    if (Pdet==0) // if they pass in a zero covariance matrix, produce exact answers.
    {
        singular = true;
        Pscale = 0;
    }
    else
    {
        Pscale = 1.0/std::pow(2.0*M_PI, double(N)/2.0)/std::sqrt(Pdet);
    }

    logPscale = -0.5 * chol.logdet() - double(N) / 2.0 * std::log(2.0*M_PI);
}

///** Return an Nx1 vector drawn from the distribution using the provided Random object **/
//template<unsigned N>
//typename c_Gaussian<N>::vector_t c_Gaussian<N>::sample(Random& r) const
//{
//    if (singular)
//        return u;
//
//    vector_t x;
//
//    for (int i = 0; i < N; i++)
//        x[i] = r.nextGaussian();
//    vector_t tmp;
//    chol.elmult(x, tmp);
//    return tmp + u;
//}

/** compute probability of Nx1 vector v **/
template<unsigned N>
double c_Gaussian<N>::unscaledProb(const vector_t& v) const
{
    if (singular)
        throw std::runtime_error("Gaussian::unscaledProb: singular");

    vector_t w;
    for (int i = 0; i < N; i++)
        w(i) = v(i) - u(i);
    vector_t t;
    chol.elsolve(w, t);
    double e = 0.;
    for (int i = 0; i < N; i++)
        e += t(i) * t(i);
    double p = std::exp(-0.5 * e);

    return p;
}

/** compute log probability of Nx1 vector v **/
template<unsigned N>
double c_Gaussian<N>::logProb(const vector_t& v) const
{
    if (singular)
        return 0;

    // special case 3x3 for performance
    // TODO: Check whether this makes sense in this templated class
    if (v.size() == 3)
    {
        const matrix_t& L = chol.getL();

        double a = v[0] - u[0];
        double b = v[1] - u[1];
        double c = v[2] - u[2];

        double y0 = a / L(0,0);
        double y1 = (b - L(1,0) * y0) / L(1,1);
        double y2 = (c - L(2,0) * y0 - L(2,1) * y1) / L(2,2);

        return logPscale - 0.5 * (y0 * y0 + y1 * y1 + y2 * y2);
    }

    vector_t w;
    for (int i = 0; i < N; i++)
        w(i) = v(i) - u(i);
    vector_t t;
    chol.elsolve(w, t);
    double e = 0.;
    for (int i = 0; i < N; i++)
        e += t(i) * t(i);
    return logPscale - 0.5 * e;
}

template<unsigned N>
double c_Gaussian<N>::getMahalanobisDistance(const vector_t& x) const
{
    if (singular)
        throw std::runtime_error("Gaussian::getMahalanobisDistance: singular");
    //vDouble dx = LinAlg::subtract(x, u);
    //return std::sqrt(LinAlg::dotProduct(dx, LinAlg::times(Pinv,dx)));

    vector_t w;
    for (int i = 0; i < N; i++)
        w(i) = x(i) - u(i);
    vector_t t;
    chol.elsolve(w, t);
    double e = 0.;
    for (int i = 0; i < N; i++)
        e += t(i) * t(i);
    return std::sqrt(e);
}

double norm_kld(const Gaussian& N0, const Gaussian& N1);

// http://ipvs.informatik.uni-stuttgart.de/mlr/marc/notes/gaussians.pdf
// "Mixture of Gaussians: Collapsing a MoG into a single Gaussian"
template<class GaussianInputIterator, class WeightInputIterator>
Gaussian collapseMixtureOfGaussian(GaussianInputIterator start, GaussianInputIterator end, WeightInputIterator weightsStart) {
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;

    if (start == end)
        return Gaussian();

    uint32_t n = start->getDimension();
    vDouble b(n, 0.0);

    auto a = start;
    auto weight = weightsStart;
    //auto weightSum = 0;
    for (; a != end; ++a, ++weight) {
        b += (*weight) * a->getMean();
        //weightSum += (*weight);
    }

    //mean /= weightSum;

    mDouble bbT = outer_prod(b, b);

    mDouble B(n, n, 0.0);

    a = start;
    weight = weightsStart;
    for (; a != end; ++a, ++weight) {
        B += (*weight) * (a->getCovariance() + outer_prod(a->getMean(), a->getMean()) - bbT);
    }

    return Gaussian(B, b);
}

template<class InputIterator>
Gaussian productOfGaussians(InputIterator start, InputIterator end) {
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;

    if (start == end)
        return Gaussian();
    uint32_t n = start->getDimension();
    uint32_t k = std::distance(start, end);
    vDouble b(n, 0.0);

    mDouble B_inv(n, n, 0.0);
    auto a = start;
    
    std::vector<mDouble> invCovs;
    invCovs.reserve(k);

    for (; a != end; ++a) {
        invCovs.push_back(mDouble(n, n));
        bool good = InvertMatrix(a->getCovariance(), invCovs.back());
        assert(good);
        B_inv += invCovs.back();
    }

    mDouble B(n, n, 0.0);
    InvertMatrix(B_inv, B);

    a = start;
    for (uint32_t i = 0; i < k; ++a, ++i) {
        b += prod(prod(B, invCovs[i]), a->getMean());
    }

    return Gaussian(B, b);
}

Gaussian productOf2Gaussians(const Gaussian& gaussianA, const Gaussian& gaussianB);


template<unsigned N>
c_Gaussian<N> productOf2Gaussians(const c_Gaussian<N>& gaussianA, const c_Gaussian<N>& gaussianB) {
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
        return c_Gaussian<N>();
    const vDouble& t1 = prod(ApBInv, a);
    const vDouble& t2 = prod(B, t1);
    const vDouble& t3 = prod(ApBInv, b);
    const vDouble& t4 = prod(A, t3);
    vDouble mean = t2 + t4;

    const mDouble& t5 = prod(ApBInv, B);
    mDouble cov = prod(A, t5);
    return c_Gaussian<N>(cov, mean);
}

template<unsigned N>
c_Gaussian<N> gaussianPoseTransform(const c_Gaussian<N>& gaussian, const boost::numeric::ublas::matrix<double>& rot, const boost::numeric::ublas::vector<double>& trans) {
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;

    assert(gaussian.getDimension() == rot.size1());
    assert(gaussian.getDimension() == rot.size2());
    assert(gaussian.getDimension() == trans.size());
    const uint32_t n = gaussian.getDimension();

    const vDouble& mu = gaussian.getMean();
    const mDouble& cov = gaussian.getCovariance();

    vDouble new_mu = prod(rot, mu) + trans;

    const mDouble& rot_trans = boost::numeric::ublas::trans(rot);
    const mDouble& cov_t1 = prod(rot,cov);
    mDouble new_cov = prod(cov_t1, rot_trans);

    return c_Gaussian<N>(new_cov, new_mu);
}

template<unsigned N>
double distanceL2(const c_Gaussian<N>& gaussianA, const c_Gaussian<N>& gaussianB) {
    typedef boost::numeric::ublas::matrix<double> mDouble;
    typedef boost::numeric::ublas::vector<double> vDouble;

    assert(gaussianA.getDimension() == gaussianB.getDimension());
    const uint32_t n = gaussianA.getDimension();

    const vDouble& a = gaussianA.getMean();
    const vDouble& b = gaussianB.getMean();
    const mDouble& A = gaussianA.getCovariance();
    const mDouble& B = gaussianB.getCovariance();

    vDouble asb = a - b;
//    vDouble asbT = /*boost::numeric::ublas::*/trans(asb);
    mDouble ApB = A + B;
    mDouble ApBInv(n, n);
    bool good = InvertMatrix(ApB, ApBInv);
    if (!good)
        return 0.0/*c_Gaussian<N>()*/;

    vDouble t1 = prod(ApBInv, asb);
    auto dist = inner_prod(asb, t1);
    return std::exp(-0.5*dist);
}
