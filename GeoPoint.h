#ifndef GEOPOINT_H
#define GEOPOINT_H

#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

class GeoPoint
{
public:
    GeoPoint();
    GeoPoint(double X, double Y);

    double getX() const;
    double getY() const;
    void setX(double X);
    void setY(double Y);
    nlohmann::json toJson() const;
    static GeoPoint fromJsonStr(std::string jsonStr);
    static GeoPoint fromJson(nlohmann::json j);
    friend std::ostream &operator<<(std::ostream &os, const GeoPoint &gp);

private:
    double X_;
    double Y_;
};

#endif