#ifndef IBVS_H
#define IBVS_H

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

typedef Eigen::Matrix<float, 8, 6> LeMat;
typedef Eigen::Matrix<float, 6, 8> LePlus;
typedef Eigen::Matrix<float, 8, 1> uv;


class IBVS {
    uv ImagePts;
    LeMat Le;
    LePlus Le_psinv, DiagMat;
    double old_z_hat;
    double Pinv_tolerance;
  public:
    void update_uv_row (int update_pair, double new_u, double new_v);
    void update_uv (std::vector<cv::Point2f> uv_new);
    void update_uv (std::vector<double> uv_new); //Overload for vector of doubles
    void update_tolerance(double newval);
    IBVS();
    void disp_uv_row(int n);
    void disp_uv();
    void update_Le_row(int, double);
    void update_Le(double);
    void display_Le();
    void MP_psinv_Le();
};

#endif // IBVS_H
