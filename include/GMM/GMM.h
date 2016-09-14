#pragma once

#include "Gaussian.h"

#include <geometry_msgs/Point.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <limits>       // std::numeric_limits
#include <math.h>

/** Representation of a Signatured Gaussian Mixture model. **/
template<unsigned DIM_S>
class GMM
{
public:
//    typedef boost::numeric::ublas::matrix<double> mDouble;
//    typedef boost::numeric::ublas::vector<double> vDouble;
    typedef boost::numeric::ublas::c_matrix<double, 3, 3> mDouble3;
    typedef boost::numeric::ublas::c_vector<double, 3> vDouble3;
    typedef boost::numeric::ublas::c_vector<double, DIM_S> Signature;

    std::vector<c_Gaussian<3>> gm;
    Signature s;
    double p;
    double weight;

    double x;
    double y;
    double z;

    std::vector<mDouble3> eVecs;
    std::vector<vDouble3> eVals;
    std::vector<geometry_msgs::Point> vizPoints;

    bool eig_valid = false;

    bool generateVizPoints(){ return eig_valid ? eig_valid : calculate_eig(); };

private:

    void init()
    {
      if (!gm.empty())
      {
        for (auto& g:gm)
        {
          x+=g.getMean()(0);
          y+=g.getMean()(1);
          z+=g.getMean()(2);
        }
        x/=((double)(gm.size()));
        y/=((double)(gm.size()));
        z/=((double)(gm.size()));
      }
      else
      {
        x = std::numeric_limits<double>::max();
        y = std::numeric_limits<double>::max();
        z = std::numeric_limits<double>::max();
      }
//      std::cout << "GMM: x: " << x << " y: " << y << std::endl;
    };

    bool calculate_eig()
    {
      eVecs.reserve(gm.size());
      eVals.reserve(gm.size());
      vizPoints.clear();
      weight = 0.0;
      for (auto& mg:gm)
      {
        eVecs.emplace_back();
        auto& eVec = eVecs.back();
        eVals.emplace_back();
        auto& eVal = eVals.back();

        // transform Eigen to OpenCV matrices
        cv::Mat m(3, 1, CV_64F);
        for (unsigned int j = 0; j < 3; ++j)
          m.at<double>(j, 0) = mg.getMean()(j);
        cv::Mat cov(3, 3, CV_64F);
        for (unsigned int j = 0; j < 3; ++j)
          for (unsigned int i = 0; i < 3; ++i)
            cov.at<double>(j, i) = mg.getCovariance()(j, i);
        // compute eigenvectors
        cv::Mat evl(1, 3, CV_64F);
        cv::Mat evt(3, 3, CV_64F);
        cv::eigen(cov, evl, evt);

        bool exception = false;
        for (unsigned int e = 0; e < 3; ++e)
        {
          if (evl.at<double>(0,e)<0.01||isnan(evl.at<double>(0,e))||isinf(evl.at<double>(0,e)))
          {
            std::cout << "evals[" << e << "] reset" << std::endl;
            evl.at<double>(0,e)=0.01;
            exception = true;
          }
        }
        if (exception)
        {
          cv::Mat evls_diag = cv::Mat::zeros(3, 3, CV_64F);
          evls_diag.at<double>(0,0) = evl.at<double>(0,0);
          evls_diag.at<double>(1,1) = evl.at<double>(0,1);
          evls_diag.at<double>(2,2) = evl.at<double>(0,2);
          cov = evt.t() * evls_diag * evt;

          boost::numeric::ublas::c_matrix<double, 3, 3> new_cov;
          for (unsigned int j = 0; j < 3; ++j)
            for (unsigned int i = 0; i < 3; ++i)
              new_cov(j, i) = cov.at<double>(j, i);
          mg = c_Gaussian<3>(new_cov, mg.getMean());

//          std::cout << "evls: " << evl.at<double>(0,0) << " " << evl.at<double>(0,1) << " " << evl.at<double>(0,2) << " ---> ";
//          cov = cv::Mat::eye(3,3,CV_64F)*0.01;
          cv::eigen(cov, evl, evt);
//          std::cout << evl.at<double>(0,0) << " " << evl.at<double>(0,1) << " " << evl.at<double>(0,2) << std::endl;
        }
        double mx = m.at<double>(0,0);
        double my = m.at<double>(1,0);
        double mz = m.at<double>(2,0);
        double min_evl = evl.at<double>(0,0);
        unsigned int min_evl_idx = 0;
        for (unsigned int e = 0; e < 3; ++e)
        {
//          if (/*evl.at<double>(0,e)<0.01||*/isnan(evl.at<double>(0,e))||isinf(evl.at<double>(0,e)))
//          {
//            evl.at<double>(0,e) = 0.01;
////            evt.at<double>(e,0) = 0.0;
////            evt.at<double>(e,1) = 0.0;
////            evt.at<double>(e,2) = 0.0;
//          }
          if (evl.at<double>(0,e)<min_evl)
          {
            min_evl = evl.at<double>(0,e);
            min_evl_idx = e;
          }
          geometry_msgs::Point a;
          geometry_msgs::Point b;
          double sigma = sqrt(evl.at<double>(0,e));
          double scale = sigma * 2.0;

          a.x = mx + evt.at<double>(e,0) * scale;
          a.y = my + evt.at<double>(e,1) * scale;
          a.z = mz + evt.at<double>(e,2) * scale;
          b.x = mx - evt.at<double>(e,0) * scale;
          b.y = my - evt.at<double>(e,1) * scale;
          b.z = mz - evt.at<double>(e,2) * scale;
//          std::cout << "(" << a.x << ", " << a.y << ", " << a.z << ") (" << b.x << ", " << b.y << ", " << b.z << ")" << std::endl;
          vizPoints.emplace_back(a);
          vizPoints.emplace_back(b);
        }

        unsigned int max_evl_idx = evl.at<double>(0,(min_evl_idx+1)%3) >= evl.at<double> (0,(min_evl_idx+2)%3) ? (min_evl_idx+1)%3 : (min_evl_idx+2)%3;
        unsigned int mid_evl_idx = 3-max_evl_idx-min_evl_idx;

        eVal[0] = evl.at<double> (0, max_evl_idx);
        eVal[1] = evl.at<double> (0, mid_evl_idx);
        eVal[2] = evl.at<double>(0, min_evl_idx);

        weight += eVal[0]*eVal[1];

        eVec(0,0) = evt.at<double>(max_evl_idx,0); eVec(0,1) = evt.at<double>(mid_evl_idx,0); eVec(0,2) = evt.at<double>(min_evl_idx,0);
        eVec(1,0) = evt.at<double>(max_evl_idx,1); eVec(1,1) = evt.at<double>(mid_evl_idx,1); eVec(1,2) = evt.at<double>(min_evl_idx,1);
        eVec(2,0) = evt.at<double>(max_evl_idx,2); eVec(2,1) = evt.at<double>(mid_evl_idx,2); eVec(2,2) = evt.at<double>(min_evl_idx,2);

//        std::vector<vDouble3> mainAxis;
//        for (unsigned int e = 0; e < 3; ++e)
//        {
//          if (e==min_evl_idx) continue;
//          double sigma = sqrt(evl.at<double>(0,e));
//          double scale = sigma * 2.0;
//          mainAxis.emplace_back(vDouble3(scale*evt.at<double>(e,0),scale*evt.at<double>(e,1),scale*evt.at<double>(e,2)));
//        }
        int count = 36;
        auto step = 2. * M_PI / double(count);
        for (auto i = 0u; i < count; ++i) {
          auto angle = i * step;
          auto next_angle = angle+step;
          geometry_msgs::Point a;
          geometry_msgs::Point b;
          a.x = mx + 2.0*sqrt(eVal[0])*eVec(0,0)*cos(angle) + 2.0*sqrt(eVal[1])*eVec(0,1)*sin(angle);
          a.y = my + 2.0*sqrt(eVal[0])*eVec(1,0)*cos(angle) + 2.0*sqrt(eVal[1])*eVec(1,1)*sin(angle);
          a.z = mz + 2.0*sqrt(eVal[0])*eVec(2,0)*cos(angle) + 2.0*sqrt(eVal[1])*eVec(2,1)*sin(angle);
          b.x = mx + 2.0*sqrt(eVal[0])*eVec(0,0)*cos(next_angle) + 2.0*sqrt(eVal[1])*eVec(0,1)*sin(next_angle);
          b.y = my + 2.0*sqrt(eVal[0])*eVec(1,0)*cos(next_angle) + 2.0*sqrt(eVal[1])*eVec(1,1)*sin(next_angle);
          b.z = mz + 2.0*sqrt(eVal[0])*eVec(2,0)*cos(next_angle) + 2.0*sqrt(eVal[1])*eVec(2,1)*sin(next_angle);
//          std::cout << "(" << a.x << ", " << a.y << ", " << a.z << ") (" << b.x << ", " << b.y << ", " << b.z << ")" << std::endl;
          vizPoints.emplace_back(a);
          vizPoints.emplace_back(b);
        }
      }

      return true;
    }

public:
    GMM(): s(boost::numeric::ublas::zero_vector<double>(DIM_S)), p(0.0), weight(0.0)
    {
      x = 0.0;
      y = 0.0;
      z = 0.0;
      init();
    };

    GMM(const GMM<DIM_S>& GMM): gm(GMM.gm), s(GMM.s), p(GMM.p), weight(GMM.weight)
    {
      init();
//      eig_valid = calculate_eig();
    };

    GMM(const std::vector<c_Gaussian<3>>& _gm, const Signature& _s=boost::numeric::ublas::zero_vector<double>(DIM_S), const double _p=0.0):
            gm(_gm), s(_s), p(_p), weight(0.0)
    {
      x = 0.0;
      y = 0.0;
      z = 0.0;

      init();

//      eig_valid = calculate_eig();
    };

    GMM(const c_Gaussian<3>& _gaussian, const Signature& _s=boost::numeric::ublas::zero_vector<double>(DIM_S), const double _p=0.0):
            s(_s), p(_p), weight(0.0)
    {
      gm.clear();
      gm.emplace_back(_gaussian);

      x = 0.0;
      y = 0.0;
      z = 0.0;

      init();

//      eig_valid = calculate_eig();
    };

    GMM(const c_Gaussian<3>& _gaussian, const double _p=0.0):
            p(_p), weight(0.0)
    {
//      std::cout << "GMM(const c_Gaussian<3>& _gaussian, const double _p=0.0)" << std::endl;
      s = boost::numeric::ublas::zero_vector<double>(DIM_S);
      gm.clear();
      gm.emplace_back(_gaussian);

      x = 0.0;
      y = 0.0;
      z = 0.0;

      init();

//      eig_valid = calculate_eig();
    };

    GMM(const std::vector<c_Gaussian<3>>& _gm, const double _p=0.0):
            p(_p), weight(0.0)
    {
//      std::cout << "GMM(const c_Gaussian<3>& _gaussian, const double _p=0.0)" << std::endl;
      s = boost::numeric::ublas::zero_vector<double>(DIM_S);
      gm.clear();
      gm.insert(gm.begin(),_gm.begin(), _gm.end());

      x = 0.0;
      y = 0.0;
      z = 0.0;

      init();

//      eig_valid = calculate_eig();
    };

    GMM(const std::vector<Gaussian>& _gm, const Signature& _s=boost::numeric::ublas::zero_vector<double>(DIM_S), const double _p=0.0):
            s(_s), p(_p), weight(0.0)
    {
      gm.clear();
      for (auto& _gaussian:_gm)
        gm.emplace_back(c_Gaussian<3>(_gaussian.getCovariance(), _gaussian.getMean()));

      x = 0.0;
      y = 0.0;
      z = 0.0;
      init();
    };

    GMM(const Gaussian& _gaussian, const Signature& _s=boost::numeric::ublas::zero_vector<double>(DIM_S), const double _p=0.0):
            s(_s), p(_p), weight(0.0)
    {
      gm.clear();
      gm.emplace_back(c_Gaussian<3>(_gaussian.getCovariance(), _gaussian.getMean()));

      x = 0.0;
      y = 0.0;
      z = 0.0;
      init();
    };

    void poseTransform(const mDouble3& rot, const vDouble3& trans)
    {
      for (auto& gaussian:gm)
      {
//        std::cout << "==========\n transform: " << trans[0] << " " << trans[1] << " " << trans[2] << std::endl;
//        std::cout << rot(0,0) << " " << rot(0,1) << " " << rot(0,2) << "\n"
//            << rot(1,0) << " " << rot(1,1) << " " << rot(1,2) << "\n"
//            << rot(2,0) << " " << rot(2,1) << " " << rot(2,2) << std::endl;
//        std::cout << "before transform: " << gaussian.getMean()[0] << " " << gaussian.getMean()[1] << " " << gaussian.getMean()[2] << std::endl;
//        std::cout << gaussian.getCovariance()(0,0) << " " << gaussian.getCovariance()(0,1) << " " << gaussian.getCovariance()(0,2) << "\n"
//            << gaussian.getCovariance()(1,0) << " " << gaussian.getCovariance()(1,1) << " " << gaussian.getCovariance()(1,2) << "\n"
//            << gaussian.getCovariance()(2,0) << " " << gaussian.getCovariance()(2,1) << " " << gaussian.getCovariance()(2,2) << std::endl;
        gaussian = gaussianPoseTransform<3>(gaussian, rot, trans);
//        std::cout << "after transform: " << gaussian.getMean()[0] << " " << gaussian.getMean()[1] << " " << gaussian.getMean()[2] << std::endl;
//        std::cout << gaussian.getCovariance()(0,0) << " " << gaussian.getCovariance()(0,1) << " " << gaussian.getCovariance()(0,2) << "\n"
//            << gaussian.getCovariance()(1,0) << " " << gaussian.getCovariance()(1,1) << " " << gaussian.getCovariance()(1,2) << "\n"
//            << gaussian.getCovariance()(2,0) << " " << gaussian.getCovariance()(2,1) << " " << gaussian.getCovariance()(2,2) << std::endl;
      }
      init();
//      eig_valid = calculate_eig();
    };

    double getX() {return x; };
    double getY() {return y; };
    double getZ() {return z; };
};

typedef GMM<4> GMM_4;
