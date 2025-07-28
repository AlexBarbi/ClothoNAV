#include "spline.h"
#include <stdexcept>
#include <algorithm>

Spline1::Spline1(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2) {
        throw std::runtime_error("Spline: x and y must have same size and at least 2 points");
    }

    std::vector<std::pair<double, double>> points;
    for (size_t i = 0; i < x.size(); ++i) {
        points.emplace_back(x[i], y[i]);
    }
    std::sort(points.begin(), points.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (size_t i = 1; i < points.size(); ++i) {
        if (points[i].first <= points[i - 1].first) {
            throw std::runtime_error("Spline: x values must be strictly increasing");
        }
    }

    x_ = std::vector<double>(points.size());
    y_ = std::vector<double>(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        x_[i] = points[i].first;
        y_[i] = points[i].second;
    }

    computeCoefficients();
}

double Spline1::operator()(double t) const {
    return evaluate(t, 0);
}

double Spline1::deriv(int k, double t) const {
    if (k < 0 || k > 2) {
        throw std::runtime_error("Spline: Derivative order must be 0, 1, or 2");
    }
    return evaluate(t, k);
}

std::pair<double, double> Spline1::getRange() const {
    return {x_.front(), x_.back()};
}

void Spline1::computeCoefficients() {
    size_t n = x_.size() - 1;
    b_.resize(n);
    c_.resize(n + 1);
    d_.resize(n);

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n + 1, n + 1);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + 1);

    A(0, 0) = 1.0;
    A(n, n) = 1.0;

    for (size_t i = 1; i < n; ++i) {
        double h_i = x_[i] - x_[i - 1];
        double h_ip1 = x_[i + 1] - x_[i];
        A(i, i - 1) = h_i;
        A(i, i) = 2.0 * (h_i + h_ip1);
        A(i, i + 1) = h_ip1;
        rhs(i) = 3.0 * ((y_[i + 1] - y_[i]) / h_ip1 - (y_[i] - y_[i - 1]) / h_i);
    }

    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(rhs);
    c_.resize(c_eigen.size());
    for (Eigen::Index i = 0; i < c_eigen.size(); ++i) {
        c_[i] = c_eigen(i);
    }

    for (size_t i = 0; i < n; ++i) {
        double h_i = x_[i + 1] - x_[i];
        b_[i] = (y_[i + 1] - y_[i]) / h_i - h_i * (c_[i + 1] + 2.0 * c_[i]) / 3.0;
        d_[i] = (c_[i + 1] - c_[i]) / (3.0 * h_i);
    }
}

double Spline1::evaluate(double t, int k) const {
    if (t < x_.front() || t > x_.back()) {
        throw std::runtime_error("Spline: Evaluation point out of range");
    }

    size_t i = 0;
    while (i < x_.size() - 1 && t > x_[i + 1]) {
        ++i;
    }
    if (i == x_.size() - 1 && t > x_.back()) {
        --i;
    }

    double dt = t - x_[i];

    if (k == 0) {
        return y_[i] + dt * (b_[i] + dt * (c_[i] + dt * d_[i]));
    } else if (k == 1) {
        return b_[i] + dt * (2.0 * c_[i] + 3.0 * d_[i] * dt);
    } else {
        return 2.0 * c_[i] + 6.0 * d_[i] * dt;
    }
}