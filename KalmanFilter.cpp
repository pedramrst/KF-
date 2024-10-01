#include "KalmanFilter.h"
#include <iostream>

KalmanFilter::KalmanFilter() {}

KalmanFilter::KalmanFilter(GeoPoint point)
    : lastObs_(point)
{
    x_.resize(4);
    x_.setZero();
    currentTime_ = KalmanFilter::getCurrentTime();
    P_ << 0.01 * 2, 0.0, 0.0, 0.0,
        0.0, 0.01 * 2, 0.0, 0.0,
        0.0, 0.0, 0.005 * 2, 0.0,
        0.0, 0.0, 0.0, 0.005 * 2;
    R_ << pow(0.01, 2), 0.0, 0.0, 0.0,
        0.0, pow(0.01, 2), 0.0, 0.0,
        0.0, 0.0, pow(0.005, 2), 0.0,
        0.0, 0.0, 0.0, pow(0.005, 2);
}

KalmanFilter::KalmanFilter(GeoPoint point, uint64_t currentTime)
    : lastObs_(point), currentTime_(currentTime)
{
    x_.resize(4);
    x_.setZero();
    P_ << 0.01 * 2, 0.0, 0.0, 0.0,
        0.0, 0.01 * 2, 0.0, 0.0,
        0.0, 0.0, 0.005 * 2, 0.0,
        0.0, 0.0, 0.0, 0.005 * 2;
    R_ << pow(0.01, 2), 0.0, 0.0, 0.0,
        0.0, pow(0.01, 2), 0.0, 0.0,
        0.0, 0.0, pow(0.005, 2), 0.0,
        0.0, 0.0, 0.0, pow(0.005, 2);
}

GeoPoint KalmanFilter::finalizeSetup(GeoPoint point)
{
    uint64_t updatedCurrentTime = KalmanFilter::getCurrentTime();
    double dt = (updatedCurrentTime - currentTime_) / 1000.0;
    x_ << point.getX(), point.getY(), (point.getX() - lastObs_.getX()) / dt, (point.getY() - lastObs_.getY()) / dt;
    currentTime_ = updatedCurrentTime;
    lastObs_ = point;
    return point;
}

GeoPoint KalmanFilter::finalizeSetup(GeoPoint point, uint64_t currentTime)
{
    uint64_t updatedCurrentTime = currentTime;
    double dt = (updatedCurrentTime - currentTime_) / 1000.0;
    x_ << point.getX(), point.getY(), (point.getX() - lastObs_.getX()) / dt, (point.getY() - lastObs_.getY()) / dt;
    currentTime_ = updatedCurrentTime;
    lastObs_ = point;
    return point;
}

GeoPoint KalmanFilter::predict() const
{
    uint64_t updatedCurrentTime = KalmanFilter::getCurrentTime();
    double dt = (updatedCurrentTime - currentTime_) / 1000.0;
    Eigen::Matrix4d A;
    A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::VectorXd prediction = A * x_;
    GeoPoint predPoint(prediction[0], prediction[1]);
    return predPoint;
}

GeoPoint KalmanFilter::update(GeoPoint point)
{
    uint64_t updatedCurrentTime = KalmanFilter::getCurrentTime();
    double dt = (updatedCurrentTime - currentTime_) / 1000.0;
    Eigen::Matrix<double, 4, 4> A;
    A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::VectorXd prediction = A * x_.cast<double>();
    Eigen::VectorXd predLocation(2);
    predLocation << prediction[0], prediction[1];
    Eigen::Matrix4d P_k = A * P_ * A.transpose();
    for (int i = 0; i < P_k.rows(); ++i)
    {
        for (int j = 0; j < P_k.cols(); ++j)
        {
            if (i != j)
            {
                P_k(i, j) = 0.0;
            }
        }
    }
    Eigen::Matrix<double, 4, 4> P_k_plus_R = P_k + R_;
    Eigen::Matrix<double, 4, 4> mask = (P_k_plus_R.array() != 0).cast<double>();
    Eigen::Matrix<double, 4, 4> K = (P_k_plus_R.array() != 0).select(P_k.array() / P_k_plus_R.array(), 0.0);
    Eigen::Vector<double, 4> obs;
    obs << point.getX(), point.getY(), (point.getX() - lastObs_.getX()) / dt, (point.getY() - lastObs_.getY()) / dt;
    Eigen::Vector<double, 4> x_k;
    x_k = prediction + K * (obs - prediction);
    P_k = (Eigen::MatrixXd::Identity(4, 4) - K) * P_k;
    currentTime_ = updatedCurrentTime;
    P_ = P_k;
    x_ = x_k;
    lastObs_ = point;
    GeoPoint updateLoc(x_k[0], x_k[1]);
    return updateLoc;
}

GeoPoint KalmanFilter::update(GeoPoint point, uint64_t currentTime)
{
    uint64_t updatedCurrentTime = currentTime;
    double dt = (updatedCurrentTime - currentTime_) / 1000.0;
    Eigen::Matrix<double, 4, 4> A;
    A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::VectorXd prediction = A * x_.cast<double>();
    Eigen::VectorXd predLocation(2);
    predLocation << prediction[0], prediction[1];
    Eigen::Matrix4d P_k = A * P_ * A.transpose();
    for (int i = 0; i < P_k.rows(); ++i)
    {
        for (int j = 0; j < P_k.cols(); ++j)
        {
            if (i != j)
            {
                P_k(i, j) = 0.0;
            }
        }
    }
    Eigen::Matrix<double, 4, 4> P_k_plus_R = P_k + R_;
    Eigen::Matrix<double, 4, 4> mask = (P_k_plus_R.array() != 0).cast<double>();
    Eigen::Matrix<double, 4, 4> K = (P_k_plus_R.array() != 0).select(P_k.array() / P_k_plus_R.array(), 0.0);
    Eigen::Vector<double, 4> obs;
    obs << point.getX(), point.getY(), (point.getX() - lastObs_.getX()) / dt, (point.getY() - lastObs_.getY()) / dt;
    Eigen::Vector<double, 4> x_k;
    x_k = prediction + K * (obs - prediction);
    P_k = (Eigen::MatrixXd::Identity(4, 4) - K) * P_k;
    currentTime_ = updatedCurrentTime;
    P_ = P_k;
    x_ = x_k;
    lastObs_ = point;
    GeoPoint updateLoc(x_k[0], x_k[1]);
    return updateLoc;
}

bool KalmanFilter::isFinilized()
{
    if (x_[0] == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

nlohmann::json KalmanFilter::toJson() const
{
    nlohmann::json j;
    j["x"] = serializeVector(x_);
    j["currentTime"] = currentTime_;
    j["P"] = serializeMatrix(P_);
    j["lastObs"] = lastObs_.toJson();
    return j;
}

KalmanFilter KalmanFilter::fromJsonStr(std::string jsonStr)
{
    nlohmann::json j = nlohmann::json::parse(jsonStr);
    KalmanFilter kf = KalmanFilter();
    kf.x_ = deserializeToEigenVector(j["x"]);
    kf.currentTime_ = j["currentTime"];
    kf.P_ = deserializeToEigenMatrix(j["P"]);
    kf.lastObs_ = GeoPoint::fromJson(j["lastObs"]);
    kf.R_ << pow(0.01, 2), 0.0, 0.0, 0.0,
        0.0, pow(0.01, 2), 0.0, 0.0,
        0.0, 0.0, pow(0.005, 2), 0.0,
        0.0, 0.0, 0.0, pow(0.005, 2);
    return kf;
}

std::string KalmanFilter::serializeVector(const Eigen::VectorXd &vec)
{
    std::ostringstream oss;
    oss << vec;
    return oss.str();
}

std::string KalmanFilter::serializeMatrix(const Eigen::Matrix4d &mat)
{
    std::ostringstream oss;
    oss << mat;
    return oss.str();
}

Eigen::VectorXd KalmanFilter::deserializeToEigenVector(const std::string &str)
{
    std::vector<double> elements;
    std::istringstream iss(str);
    double element;
    while (iss >> element)
    {
        elements.push_back(element);
    }
    Eigen::VectorXd vec(elements.size());
    for (size_t i = 0; i < elements.size(); ++i)
    {
        vec(i) = elements[i];
    }
    return vec;
}

Eigen::Matrix4d KalmanFilter::deserializeToEigenMatrix(const std::string &str)
{
    Eigen::Matrix4d mat;
    std::istringstream iss(str);
    for (int i = 0; i < mat.rows(); ++i)
    {
        for (int j = 0; j < mat.cols(); ++j)
        {
            if (!(iss >> mat(i, j)))
            {
                throw std::runtime_error("(KalmanFilter) Error deserializing matrix.");
            }
        }
    }
    return mat;
}

uint64_t KalmanFilter::getCurrentTime()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

Eigen::VectorXd KalmanFilter::getX() const
{
    return x_;
}