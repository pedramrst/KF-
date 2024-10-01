#include "GeoPoint.h"

GeoPoint::GeoPoint() {}

GeoPoint::GeoPoint(double X, double Y)
    : X_(X), Y_(Y) {}

double GeoPoint::getX() const
{
    return X_;
}

double GeoPoint::getY() const
{
    return Y_;
}

void GeoPoint::setX(double X)
{
    X_ = X;
}

void GeoPoint::setY(double Y)
{
    Y_ = Y;
}

nlohmann::json GeoPoint::toJson() const
{
    nlohmann::json j = {{"X", X_}, {"Y", Y_}};
    return j;
}

GeoPoint GeoPoint::fromJsonStr(std::string jsonStr)
{
    nlohmann::json j = nlohmann::json::parse(jsonStr);
    GeoPoint geoPoint(j["X"].get<double>(), j["Y"].get<double>());
    return geoPoint;
}

GeoPoint GeoPoint::fromJson(nlohmann::json j)
{
    GeoPoint geoPoint(j["X"].get<double>(), j["Y"].get<double>());
    return geoPoint;
}

std::ostream &operator<<(std::ostream &os, const GeoPoint &gp)
{
    os << "GeoPoint(X: " << gp.X_ << ", Y: " << gp.Y_ << ")";
    return os;
}