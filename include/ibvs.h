#ifndef IBVS_H
#define IBVS_H

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

typedef Eigen::Matrix<float, 8, 6> LeMat;
typedef Eigen::Matrix<float, 6, 8> LePlus;
typedef Eigen::Matrix<float, 8, 1> uv;
typedef Eigen::Matrix<float, 6, 1> velocity;

class IBVS {
    uv ImagePts;
    LeMat Le;
    LePlus Le_psinv, DiagMat;
    //focal length is measured in pixels. bsln is in meters, and represents the line distance separating the two points in our z_est calculation
    double old_z_hat, z_est, focal_lngth, bsln;
    double Pinv_tolerance;
    velocity vc;

    //function declarations

    //void update_z_est();
  public:
    void update_z_est(std::vector<cv::Point2f> pts);
    void update_uv_row (int update_pair, double new_u, double new_v);
    void update_uv (std::vector<cv::Point2f> uv_new);
    void update_uv (std::vector<double> uv_new); //Overload for vector of doubles
    void update_tolerance(double newval);
    IBVS(double baseline = 0.15, double focal_length = 700.00);
    void disp_uv_row(int n);
    void disp_uv();
    void update_Le_row(int, double);
    void update_Le(double);
    void display_Le();
    void MP_psinv_Le();
    velocity calculate_vc();
};

#endif // IBVS_H
