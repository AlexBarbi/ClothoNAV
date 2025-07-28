#ifndef SPLINE_H
#define SPLINE_H

#include <Eigen/Dense>
#include <vector>
#include <utility>

class Spline1 {
public:
    Spline1(const std::vector<double>& x, const std::vector<double>& y);

    double operator()(double t) const;
    double deriv(int k, double t) const;
    std::pair<double, double> getRange() const;

private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;

    void computeCoefficients();
    double evaluate(double t, int k) const;
};

#endif // SPLINE_H