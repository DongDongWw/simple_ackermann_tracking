#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

namespace willand_ackermann {

class QuinticCurveGenerator {
public:
  typedef Eigen::Vector2d Point2D;

  std::vector<Point2D> getGlobalPath(const Point2D &start, const Point2D &end,
                                     double interval) {
    start_ = start;
    end_ = end;
    generateRandomCoefficients();
    points_ = std::vector<Point2D>();
    double t = 0.0;
    while (std::abs(t - 1.0) > 1e-6) {
      points_.push_back(getPoint(t));
      double next_t = findParameterForArcLength(interval, t);
      t = next_t;
    }
    points_.push_back(getPoint(t));
    return points_;
  }

  std::vector<Point2D> generateReferenceTrajectory(const Point2D &p,
                                                   int number_of_points) {
    double min_dist = std::numeric_limits<double>::max();
    int min_dist_idx = 0;
    for (size_t i = 0; i < points_.size(); ++i) {
      double dist = (points_.at(i) - p).norm();
      if (dist < min_dist) {
        min_dist = dist;
        min_dist_idx = i;
      }
    }
    std::vector<Point2D> ref_traj(number_of_points, points_.back());
    for (size_t i = min_dist_idx;
         i < min_dist_idx + number_of_points && i < points_.size(); ++i) {
      ref_traj.at(i - min_dist_idx) = points_.at(i);
    }
    return ref_traj;
  }

private:
  Point2D start_;
  Point2D end_;
  std::vector<Point2D> points_;
  Eigen::Matrix<double, 6, 2> coefficients_;

  void generateRandomCoefficients() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-10.0, 10.0);

    for (int dim = 0; dim < 2; ++dim) {
      coefficients_(0, dim) = start_(dim);
      if (dim == 0) {
        coefficients_(1, dim) = std::abs(dis(gen));
      } else {
        coefficients_(1, dim) = 0;
      }
      coefficients_(2, dim) = dis(gen);
      coefficients_(3, dim) = dis(gen);
      coefficients_(4, dim) = dis(gen);
      coefficients_(5, dim) = end_(dim) - coefficients_(0, dim) -
                              coefficients_(1, dim) - coefficients_(2, dim) -
                              coefficients_(3, dim) - coefficients_(4, dim);
    }
  }

  Point2D getPoint(double t) const {
    Eigen::VectorXd powers(6);
    powers << 1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t;
    return coefficients_.transpose() * powers;
  }

  Point2D getDerivative(double t) const {
    Eigen::VectorXd powers(6);
    powers << 0, 1, 2 * t, 3 * t * t, 4 * t * t * t, 5 * t * t * t * t;
    return coefficients_.transpose() * powers;
  }

  double calculateArcLength(double t0, double t1, int num_samples = 100) const {
    double length = 0.0;
    double dt = (t1 - t0) / num_samples;
    for (int i = 0; i < num_samples; ++i) {
      double t = t0 + i * dt;
      Point2D derivative = getDerivative(t);
      length += derivative.norm() * dt;
    }
    return length;
  }

  double findParameterForArcLength(double target_length, double start,
                                   int num_samples = 100) const {
    double low = start, high = 1.0;
    double length = calculateArcLength(start, high, num_samples);
    if (length < target_length) {
      return high;
    }
    while (high - low > 1e-6) {
      double mid = (low + high) / 2.0;
      double length = calculateArcLength(start, mid, num_samples);
      if (length < target_length) {
        low = mid;
      } else {
        high = mid;
      }
    }
    return (low + high) / 2.0;
  }
};
}; // namespace willand_ackermann