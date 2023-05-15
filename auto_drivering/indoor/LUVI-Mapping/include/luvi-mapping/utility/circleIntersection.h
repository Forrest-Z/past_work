//
// Created by xc on 2021/3/24.
//

#ifndef ALOAM_VELODYNE_CIRCLEINTERSECTION_H
#define ALOAM_VELODYNE_CIRCLEINTERSECTION_H

#include <math.h>
#include <vector>
#include <array>

namespace LIRO{
    class Point{
    public:
        Point(double x,double y):x_(x),y_(y){}
        double get_x_coord(){ return x_;}
        double get_y_coord(){return y_;}
    private:
        double x_,y_;

    };


    class Circle{
    public:
        Circle(double x,double y,double r):center_(x,y),radius_(r){}
        ~Circle(){}
        double get_radius()const {return radius_;}
        Point get_center() const { return center_;}
        std::vector<Point> FindIntersectionPoints(const Circle& other){
            std::vector<Point> result;
            double dx = other.get_center().get_x_coord() - center_.get_x_coord();
            double dy = other.get_center().get_y_coord() - center_.get_y_coord();
            double d = hypot(dx,dy);
            if(d > radius_ + other.get_radius())
                return result;
            if(d < std::fabs(radius_ - other.get_radius()))
                return  result;
            double a = ((radius_ * radius_) - (other.get_radius() * other.get_radius()) + (d * d)) / (2.0 * d);
            double x2 = center_.get_x_coord() + (dx * a/ d);
            double y2 = center_.get_y_coord() + (dy * a/ d);
            double h = std::sqrt((radius_ * radius_)-(a * a));
            double rx = -dy * (h /d);
            double ry = -dx * (h /d);

            double p1x = x2 + rx;
            double p2x = x2 - rx;
            double p1y = y2 + ry;
            double p2y = y2 - ry;

            if(p1x == p2x && p1y==p2y)
                result.push_back(Point(p1x,p1y));
            else{
                result.push_back(Point(p1x,p1y));
                result.push_back(Point(p2x,p2y));
            }
            return result;
        }

    private:
        Point center_;
        double radius_;
    };

} // namespace LIRO

#endif //ALOAM_VELODYNE_CIRCLEINTERSECTION_H
