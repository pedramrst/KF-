#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "GeoPoint.h"

class KalmanFilter
{
public:
    KalmanFilter();
    KalmanFilter(GeoPoint point);
    KalmanFilter(GeoPoint point, uint64_t currentTime);

    GeoPoint finalizeSetup(GeoPoint point);
    GeoPoint finalizeSetup(GeoPoint point, uint64_t currentTime);
    GeoPoint predict() const;
    GeoPoint update(GeoPoint point);
    GeoPoint update(GeoPoint point, uint64_t currentTime);
    bool isFinilized();
    nlohmann::json toJson() const;
    Eigen::VectorXd getX() const;
    static KalmanFilter fromJsonStr(std::string jsonStr);

private:
    Eigen::VectorXd x_;
    uint64_t currentTime_;
    Eigen::Matrix4d P_;
    GeoPoint lastObs_;
    Eigen::Matrix4d R_;

    static std::string serializeVector(const Eigen::VectorXd &vec);
    static std::string serializeMatrix(const Eigen::Matrix4d &mat);
    static Eigen::VectorXd deserializeToEigenVector(const std::string &str);
    static Eigen::Matrix4d deserializeToEigenMatrix(const std::string &str);
    static uint64_t getCurrentTime();
};

#endif