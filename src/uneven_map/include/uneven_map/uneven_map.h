#pragma once

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <random>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

namespace uneven_planner
{
    struct RXS2
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        double z;
        double sigma;
        Eigen::Vector3d assess;//roughness flatness sparsity;
        Eigen::Vector2d zb;
        
        RXS2(): z(0.0), sigma(0.0), zb(Eigen::Vector2d::Zero()), assess(Eigen::Vector3d::Zero()) {}
        RXS2(double z_, double sigma_, Eigen::Vector2d zb_,Eigen::Vector3d assess_): z(z_), sigma(sigma_), zb(zb_), assess(assess_){}
        inline double getC() { return sqrt(1.0 - zb(0)*zb(0) - zb(1)*zb(1)); }
        inline double getCosXi() { return sqrt(1.0 - zb(0)*zb(0) - zb(1)*zb(1)); }
        inline RXS2 operator+(const RXS2& a)
        {
            return RXS2(z+a.z, sigma+a.sigma, zb+a.zb, assess+a.assess);
        }
        inline RXS2 operator-(const RXS2& a)
        {
            return RXS2(z-a.z, sigma-a.sigma, zb-a.zb, assess-a.assess);
        }
        inline RXS2 operator*(const double& a)
        {
            return RXS2(z*a, sigma*a, zb*a, assess*a);
        }
        inline Eigen::Vector3d toVector()
        {
            return Eigen::Vector3d(sigma, zb.x(), zb.y());
        }
    };

    class UnevenMap
    {
        private:
            // params
            bool            show_zbso2;
            int             iter_num;
            int             show_type;
            int             maxpt_size;
            double          ellipsoid_x;
            double          ellipsoid_y;
            double          ellipsoid_z;
            double          xy_resolution, xy_resolution_inv;
            double          yaw_resolution, yaw_resolution_inv;
            double          min_cnormal;
            double          max_sparsity;
            double          offset;
            double          flat_r;
            double          lamda1;
            double          lamda2;
            double          lamda3;
            double          lamda4;
            double          max_rho;
            double          gravity;
            double          mass;
            static int      lowlim;
            static int      uplim;
            static Eigen::Vector4d lamda;
            Eigen::Vector3d map_origin;
            Eigen::Vector3d map_size;
            Eigen::Vector3d min_boundary;
            Eigen::Vector3d max_boundary;
            Eigen::Vector3i min_idx;
            Eigen::Vector3i max_idx;
            Eigen::Vector3i voxel_num;
            Eigen::Vector3i pt_blue = Eigen::Vector3i(43,115,175);
            Eigen::Vector3i pt_yellow = Eigen::Vector3i(252,211,55);
            Eigen::Vector3i pt_red = Eigen::Vector3i(222,28,49);
            Eigen::Vector3i pt_green = Eigen::Vector3i(87,149,114);
            Eigen::Vector3i pt_purple = Eigen::Vector3i(139,38,113);
            string          map_file;
            string          pcd_file;

            //datas
            vector<RXS2>    map_buffer;
            vector<double>  c_buffer;
            vector<char>    occ_buffer;
            vector<char>    occ_r2_buffer;
            pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud;
            pcl::PointCloud<pcl::PointXY>::Ptr world_cloud_plane;
            pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud_collision;
            pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
            pcl::KdTreeFLANN<pcl::PointXY> kd_tree_plane;

            //ros
            ros::Publisher                  origin_pub;
            ros::Publisher                  filtered_pub;
            ros::Publisher                  collision_pub;
            ros::Publisher                  zb_pub;
            ros::Publisher                  so2_test_pub;
            ros::Publisher                  iter_pub;
            ros::Timer                      vis_timer;
            sensor_msgs::PointCloud2        origin_cloud_msg;
            sensor_msgs::PointCloud2        filtered_cloud_msg;
            sensor_msgs::PointCloud2        collision_cloud_msg;
            visualization_msgs::MarkerArray so2_test_msg;
            visualization_msgs::MarkerArray zb_msg; 
            visualization_msgs::MarkerArray iter_msg;   
            bool                            map_ready = false;

        public:
            UnevenMap() {}
            ~UnevenMap() {}

            static RXS2 filter(Eigen::Vector3d pos, vector<Eigen::Vector3d>& points1 ,double deltah);
            static Eigen::Matrix3d skewSym(Eigen::Vector3d vec);
            static double calYawFromR(Eigen::Matrix3d R);
            static void normSO2(double& yaw);

            void init(ros::NodeHandle& nh);
            bool constructMapInput();
            bool constructMap();
            void visCallback(const ros::TimerEvent& /*event*/);

            inline void getTerrain(const Eigen::Vector3d& pos, RXS2& value);
            inline void getTerrainPos(const Eigen::Vector3d& pos, Eigen::Matrix3d& R, Eigen::Vector3d& p);
            // invCosVphix, sinPhix, invCosVphiy, sinPhiy, cosXi, invCosXi, sigma
            inline void getTerrainVariables(const Eigen::Vector3d& pos, vector<double>& values);
            inline void getTerrainWithGradI(const Eigen::Vector3d& pos, RXS2& value, Eigen::Matrix<double, 4, 3>& grad);
            // invCosVphix, sinPhix, invCosVphiy, sinPhiy, cosXi, invCosXi, sigma
            inline void getAllWithGrad(const Eigen::Vector3d& pos, vector<double>& values, vector<Eigen::Vector3d>& grads);
            inline void getTerrainWithGradz(const Eigen::Vector3d& pos, RXS2& value, Eigen::Matrix<double,1,3>& grad);
            
            inline double getGravity(void);
            inline double getMass(void);
            inline double getTerrainSig(const Eigen::Vector3d& pos);
            inline void boundIndex(Eigen::Vector3i& id);
            inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
            inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
            inline int toAddress(const Eigen::Vector3i& id);
            inline int toAddress(const int& x, const int& y, const int& yaw);
            inline bool isInMap(const Eigen::Vector3d& pos);
            inline bool isInMap(const Eigen::Vector3i& idx);
            inline int isOccupancy(const Eigen::Vector3d& pos);
            inline int isOccupancy(const Eigen::Vector3i& id);
            inline int isOccupancyXY(const Eigen::Vector3d& pxy);
            inline int getXYNum();
            inline bool mapReady();
            inline bool iscollisionFress(const Eigen::Vector3d& pos);
            inline bool isinBox(const Eigen::Vector3d& pos);
            inline void pointSeg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g);

            typedef shared_ptr<UnevenMap> Ptr;
            typedef unique_ptr<UnevenMap> UniPtr;
    };

    inline void UnevenMap::getTerrain(const Eigen::Vector3d& pos, RXS2& value)
    {
        if (!isInMap(pos))
        {
            value = RXS2();
            ROS_WARN("[Uneven Map] pos isn't in map, check it!");
            return;
        }

        Eigen::Vector3d pos_m = pos;
        pos_m(0) -= 0.5 * xy_resolution;
        pos_m(1) -= 0.5 * xy_resolution;
        pos_m(2) -= 0.5 * yaw_resolution;
        normSO2(pos_m(2));

        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector3d diff = pos - idx_pos;
        diff(0) *= xy_resolution_inv;
        diff(1) *= xy_resolution_inv;
        // SO(2) process
        diff(2) = atan2(sin(pos(2)-idx_pos(2)), cos(pos(2)-idx_pos(2))) * yaw_resolution_inv;

        RXS2 values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int yaw = 0; yaw < 2; yaw++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, yaw);
                    boundIndex(current_idx);
                    values[x][y][yaw] = map_buffer[toAddress(current_idx)];
                }

        // value
        RXS2 v00 = values[0][0][0] * (1 - diff[0]) + values[1][0][0] * diff[0];
        RXS2 v01 = values[0][0][1] * (1 - diff[0]) + values[1][0][1] * diff[0];
        RXS2 v10 = values[0][1][0] * (1 - diff[0]) + values[1][1][0] * diff[0];
        RXS2 v11 = values[0][1][1] * (1 - diff[0]) + values[1][1][1] * diff[0];
        RXS2 v0 = v00 * (1 - diff[1]) + v10 * diff[1];
        RXS2 v1 = v01 * (1 - diff[1]) + v11 * diff[1];
        value = v0 * (1 - diff[2]) + v1 * diff[2];

        return;
    }

    inline void UnevenMap::getTerrainPos(const Eigen::Vector3d& pos, Eigen::Matrix3d& R, Eigen::Vector3d& p)
    {
        RXS2 rs2;
        getTerrain(pos, rs2);

        Eigen::Vector3d zb(rs2.zb.x(), rs2.zb.y(), rs2.getC());
        Eigen::Vector3d xyaw(cos(pos(2)), sin(pos(2)), 0.0);

        R.col(2) = zb;
        R.col(1) = zb.cross(xyaw).normalized();
        R.col(0) = R.col(1).cross(zb);
        p = pos;
        p(2) = rs2.z;

        return;
    }
    
    // invCosVphix, sinPhix, invCosVphiy, sinPhiy, cosXi, invCosXi, sigma
    inline void UnevenMap::getTerrainVariables(const Eigen::Vector3d& pos, vector<double>& values)
    {
        double inv_cos_vphix, sin_phix, inv_cos_vphiy, sin_phiy, cos_xi, inv_cos_xi;

        RXS2 rs2;
        getTerrain(pos, rs2);

        double c = rs2.getC();
        double inv_c = 1.0 / c;
        double cyaw = cos(pos(2));
        double syaw = sin(pos(2));
        Eigen::Vector2d xyaw(cyaw, syaw);
        Eigen::Vector2d yyaw(-syaw, cyaw);
        double t = xyaw.dot(rs2.zb);
        double s = -yyaw.dot(rs2.zb);
        double sqrt_1_t2 = sqrt(1.0 - t*t);
        double inv_sqrt_1_t2 = 1.0 / sqrt_1_t2;
        double inv_sqrt_1_t2_3 = inv_sqrt_1_t2 * inv_sqrt_1_t2 * inv_sqrt_1_t2;

        inv_cos_vphix = inv_sqrt_1_t2;
        sin_phix = -c * t * inv_sqrt_1_t2;
        inv_cos_vphiy = sqrt_1_t2 * inv_c;
        sin_phiy = s * inv_sqrt_1_t2;
        cos_xi = c;
        inv_cos_xi = inv_c; 

        values.clear();

        values.push_back(inv_cos_vphix);
        values.push_back(sin_phix);
        values.push_back(inv_cos_vphiy);
        values.push_back(sin_phiy);
        values.push_back(cos_xi);
        values.push_back(inv_cos_xi);
        values.push_back(rs2.sigma);
    }

    inline void UnevenMap::getTerrainWithGradI(const Eigen::Vector3d& pos, RXS2& value, Eigen::Matrix<double, 4, 3>& grad)
    {
        if (!isInMap(pos))
        {
            grad.setZero();
            value = RXS2();
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector3d pos_m = pos;
        pos_m(0) -= 0.5 * xy_resolution;
        pos_m(1) -= 0.5 * xy_resolution;
        pos_m(2) -= 0.5 * yaw_resolution;
        normSO2(pos_m(2));

        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector3d diff = pos - idx_pos;
        diff(0) *= xy_resolution_inv;
        diff(1) *= xy_resolution_inv;
        // SO(2) process
        diff(2) = atan2(sin(pos(2)-idx_pos(2)), cos(pos(2)-idx_pos(2))) * yaw_resolution_inv;

        RXS2 values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int yaw = 0; yaw < 2; yaw++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, yaw);
                    boundIndex(current_idx);
                    values[x][y][yaw] = map_buffer[toAddress(current_idx)];
                }
        
        // value & grad
        RXS2 v00 = values[0][0][0] * (1 - diff[0]) + values[1][0][0] * diff[0];
        RXS2 v01 = values[0][0][1] * (1 - diff[0]) + values[1][0][1] * diff[0];
        RXS2 v10 = values[0][1][0] * (1 - diff[0]) + values[1][1][0] * diff[0];
        RXS2 v11 = values[0][1][1] * (1 - diff[0]) + values[1][1][1] * diff[0];
        RXS2 v0 = v00 * (1 - diff[1]) + v10 * diff[1];
        RXS2 v1 = v01 * (1 - diff[1]) + v11 * diff[1];
        value = v0 * (1 - diff[2]) + v1 * diff[2];

        grad.block<3, 1>(0, 2) = (v1 - v0).toVector() * yaw_resolution_inv;
        grad.block<3, 1>(0, 1) = ((v10 - v00) * (1 - diff[2]) + (v11 - v01) * diff[2]).toVector() * xy_resolution_inv;
        grad.block<3, 1>(0, 0) = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]).toVector();
        grad.block<3, 1>(0, 0) += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]).toVector();
        grad.block<3, 1>(0, 0) += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]).toVector();
        grad.block<3, 1>(0, 0) += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]).toVector();
        grad.block<3, 1>(0, 0) *= xy_resolution_inv;
        grad.row(3) = -(grad.row(1)*value.zb.x() + grad.row(2)*value.zb.y()) / value.getC();

        return;
    }

    inline void UnevenMap::getTerrainWithGradz(const Eigen::Vector3d& pos, RXS2& value, Eigen::Matrix<double,1,3>& grad)
    {
        if (!isInMap(pos))
        {
            grad.setZero();
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector3d pos_m = pos;
        pos_m(0) -= 0.5 * xy_resolution;
        pos_m(1) -= 0.5 * xy_resolution;
        pos_m(2) -= 0.5 * yaw_resolution;
        normSO2(pos_m(2));

        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector3d diff = pos - idx_pos;
        diff(0) *= xy_resolution_inv;
        diff(1) *= xy_resolution_inv;
        // SO(2) process
        diff(2) = atan2(sin(pos(2)-idx_pos(2)), cos(pos(2)-idx_pos(2))) * yaw_resolution_inv;

        RXS2 values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int yaw = 0; yaw < 2; yaw++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, yaw);
                    boundIndex(current_idx);
                    values[x][y][yaw] = map_buffer[toAddress(current_idx)];
                }
        
        // value & grad
        RXS2 v00 = values[0][0][0] * (1 - diff[0]) + values[1][0][0] * diff[0];
        RXS2 v01 = values[0][0][1] * (1 - diff[0]) + values[1][0][1] * diff[0];
        RXS2 v10 = values[0][1][0] * (1 - diff[0]) + values[1][1][0] * diff[0];
        RXS2 v11 = values[0][1][1] * (1 - diff[0]) + values[1][1][1] * diff[0];
        RXS2 v0 = v00 * (1 - diff[1]) + v10 * diff[1];
        RXS2 v1 = v01 * (1 - diff[1]) + v11 * diff[1];
        value = v0 * (1 - diff[2]) + v1 * diff[2];

        grad(0, 2) = (v1 - v0).z * yaw_resolution_inv;
        grad(0, 1) = ((v10 - v00) * (1 - diff[2]) + (v11 - v01) * diff[2]).z * xy_resolution_inv;
        grad(0, 0) = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]).z;
        grad(0, 0) += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]).z;
        grad(0, 0) += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]).z;
        grad(0, 0) += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]).z;
        grad(0, 0) *= xy_resolution_inv;
        return;
    }
    // getAllWithGrad: get values and grads of {invCosVphix, sinPhix, invCosVphiy, sinPhiy, cosXi, invCosXi, sigma}
    inline void UnevenMap::getAllWithGrad(const Eigen::Vector3d& pos, vector<double>& values, vector<Eigen::Vector3d>& grads)
    {
        double inv_cos_vphix, sin_phix, inv_cos_vphiy, sin_phiy, cos_xi, inv_cos_xi;
        Eigen::Vector3d grad_inv_cos_vphix, grad_sin_phix, grad_inv_cos_vphiy, grad_sin_phiy, grad_cos_xi, grad_inv_cos_xi;

        RXS2 rs2;
        Eigen::Matrix<double, 4, 3> rs2_grad;

        getTerrainWithGradI(pos, rs2, rs2_grad);
        double c = rs2.getC();
        double inv_c = 1.0 / c;
        double cyaw = cos(pos(2));
        double syaw = sin(pos(2));
        Eigen::Vector2d xyaw(cyaw, syaw);
        Eigen::Vector2d yyaw(-syaw, cyaw);
        double t = xyaw.dot(rs2.zb);
        double s = -yyaw.dot(rs2.zb);
        double sqrt_1_t2 = sqrt(1.0 - t*t);
        double inv_sqrt_1_t2 = 1.0 / sqrt_1_t2;
        double inv_sqrt_1_t2_3 = inv_sqrt_1_t2 * inv_sqrt_1_t2 * inv_sqrt_1_t2;
        Eigen::Vector3d dt = rs2_grad.block<2, 3>(1, 0).transpose()*xyaw;
        Eigen::Vector3d ds = -rs2_grad.block<2, 3>(1, 0).transpose()*yyaw;
        dt(2) -= s;
        ds(2) += t;

        inv_cos_vphix = inv_sqrt_1_t2;
        sin_phix = -c * t * inv_sqrt_1_t2;
        inv_cos_vphiy = sqrt_1_t2 * inv_c;
        sin_phiy = s * inv_sqrt_1_t2;
        cos_xi = c;
        inv_cos_xi = inv_c; 

        grad_inv_cos_vphix = t * inv_sqrt_1_t2_3 * dt;
        grad_sin_phix = -(t * inv_sqrt_1_t2 * rs2_grad.row(3).transpose() + inv_sqrt_1_t2_3 * c * dt);
        grad_inv_cos_vphiy = -inv_c * (t * inv_sqrt_1_t2 * dt + sqrt_1_t2 * inv_c * rs2_grad.row(3).transpose());
        grad_sin_phiy = inv_sqrt_1_t2 * ds + t * inv_sqrt_1_t2_3 * s * dt;
        grad_cos_xi = rs2_grad.row(3);
        grad_inv_cos_xi = -inv_cos_xi * inv_cos_xi * rs2_grad.row(3);
        
        values.clear();
        grads.clear();

        values.push_back(inv_cos_vphix);
        values.push_back(sin_phix);
        values.push_back(inv_cos_vphiy);
        values.push_back(sin_phiy);
        values.push_back(cos_xi);
        values.push_back(inv_cos_xi);
        values.push_back(rs2.sigma);

        grads.push_back(grad_inv_cos_vphix);
        grads.push_back(grad_sin_phix);
        grads.push_back(grad_inv_cos_vphiy);
        grads.push_back(grad_sin_phiy);
        grads.push_back(grad_cos_xi);
        grads.push_back(grad_inv_cos_xi);
        grads.push_back(rs2_grad.row(0));

        return;
    }

    inline double UnevenMap::getGravity(void)
    {
        return gravity;
    }

    inline double UnevenMap::getMass(void)
    {
        return mass;
    }

    inline double UnevenMap::getTerrainSig(const Eigen::Vector3d& pos)
    {
        RXS2 value;       
        getTerrain(pos, value);

        return value.sigma;
    }

    inline void UnevenMap::boundIndex(Eigen::Vector3i& id)
    {
        id(0) = max(min(id(0), max_idx(0)), min_idx(0));
        id(1) = max(min(id(1), max_idx(1)), min_idx(1));
        // SO(2) process
        while (id(2) > max_idx(2))
            id(2) -= voxel_num(2);
        while (id(2) < min_idx(2))
            id(2) += voxel_num(2);

        return;
    }

    inline void UnevenMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
    {
        id(0) = floor((pos(0) - map_origin(0)) * xy_resolution_inv);
        id(1) = floor((pos(1) - map_origin(1)) * xy_resolution_inv);
        id(2) = floor((pos(2) - map_origin(2)) * yaw_resolution_inv);
        return;
    }

    inline void UnevenMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
    {
        pos(0) = (id(0) + 0.5) * xy_resolution  + map_origin(0);
        pos(1) = (id(1) + 0.5) * xy_resolution  + map_origin(1);
        pos(2) = (id(2) + 0.5) * yaw_resolution + map_origin(2);
        return;
    }

    inline int UnevenMap::toAddress(const Eigen::Vector3i& id) 
    {
        return id(0) * voxel_num(1)*voxel_num(2) + id(1) * voxel_num(2) + id(2);
    }

    inline int UnevenMap::toAddress(const int& x, const int& y, const int& yaw) 
    {
        return x * voxel_num(1)*voxel_num(2) + y * voxel_num(2) + yaw;
    }
    
    inline bool UnevenMap::isInMap(const Eigen::Vector3d& pos) 
    {
        if (pos(0) < min_boundary(0) + 1e-4 || \
            pos(1) < min_boundary(1) + 1e-4    ) 
        {
            return false;
        }

        if (pos(0) > max_boundary(0) - 1e-4 || \
            pos(1) > max_boundary(1) - 1e-4    ) 
        {
            return false;
        }
        if (pos(2) < min_boundary(2) + 1e-4 || \
            pos(2) > max_boundary(2) - 1e-4    )
        {
            std::cout<<"pos2 is wrong"<<std::endl;
        }
        return true;
    }

    inline bool UnevenMap::isInMap(const Eigen::Vector3i& idx)
    {
        if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
        {
            return false;
        }

        if (idx(0) > voxel_num(0) - 1 || \
            idx(1) > voxel_num(1) - 1 || \
            idx(2) > voxel_num(2) - 1     ) 
        {
            return false;
        }

        return true;
    }

    inline int UnevenMap::isOccupancy(const Eigen::Vector3d& pos)
    {
        Eigen::Vector3i id;

        posToIndex(pos, id);
        
        return isOccupancy(id);
    }

    inline int UnevenMap::isOccupancy(const Eigen::Vector3i& id)
    {
        if (!isInMap(id))
            return -1;

        return int(occ_buffer[toAddress(id)]);
    }

    inline int UnevenMap::isOccupancyXY(const Eigen::Vector3d& pxy)
    {
        Eigen::Vector3i id;

        posToIndex(pxy, id);

        if (!isInMap(id))
            return -1;

        return int(occ_r2_buffer[id(0)*voxel_num(1) + id(1)]);
    }

    inline int UnevenMap::getXYNum()
    {
        return voxel_num(0)*voxel_num(1);
    }

    inline bool UnevenMap::mapReady()
    {
        return map_ready;
    }

    inline void UnevenMap::pointSeg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g)
    {
        pcl::PointCloud<pcl::PointXYZ> collision1,collision2,collision3,collision4,collision5,collision6,collision7,collision8,collision9,collision10;
        
        //1
        pcl::CropBox<pcl::PointXYZ> clipper_collision1;
        clipper_collision1.setMin(Eigen::Vector4f(-2.3, -4.5, 0.207, 1.0));
        clipper_collision1.setMax(Eigen::Vector4f(-1.8, -3.86, 3.0, 1.0));
        clipper_collision1.setInputCloud(cloud_c);
        clipper_collision1.setNegative(false);
        clipper_collision1.filter(collision1);        

        pcl::CropBox<pcl::PointXYZ> clipper_ground1;
        pcl::PointCloud<pcl::PointXYZ> temp1;
        clipper_ground1.setMin(Eigen::Vector4f(-2.3, -4.5, 0.207, 1.0));
        clipper_ground1.setMax(Eigen::Vector4f(-1.8, -3.86, 3.0, 1.0));
        clipper_ground1.setInputCloud(cloud_c);
        clipper_ground1.setNegative(true);
        clipper_ground1.filter(temp1);       
        //cloud_g->clear();

        //2
        pcl::CropBox<pcl::PointXYZ> clipper_collision2;
        clipper_collision2.setMin(Eigen::Vector4f(-3.424, -3.2, 0.035, 1.0));
        clipper_collision2.setMax(Eigen::Vector4f(-2.892, -2.502, 3.0, 1.0));
        clipper_collision2.setInputCloud(cloud_c);
        clipper_collision2.setNegative(false);
        clipper_collision2.filter(collision2);

        pcl::CropBox<pcl::PointXYZ> clipper_ground2;
        pcl::PointCloud<pcl::PointXYZ> temp2;
        clipper_ground2.setMin(Eigen::Vector4f(-3.424, -3.2, 0.035, 1.0));
        clipper_ground2.setMax(Eigen::Vector4f(-2.892, -2.502, 3.0, 1.0));
        clipper_ground2.setInputCloud(temp1.makeShared());
        clipper_ground2.setNegative(true);
        clipper_ground2.filter(temp2);      
        temp1.clear();
        //3
        pcl::CropBox<pcl::PointXYZ> clipper_collision3;
        clipper_collision3.setMin(Eigen::Vector4f(0.25, -4.04, 0.254, 1.0));
        clipper_collision3.setMax(Eigen::Vector4f(0.673, -3.5, 3.0, 1.0));
        clipper_collision3.setInputCloud(cloud_c);
        clipper_collision3.setNegative(false);
        clipper_collision3.filter(collision3);        

        pcl::PointCloud<pcl::PointXYZ> temp3;
        pcl::CropBox<pcl::PointXYZ> clipper_ground3;
        clipper_ground3.setMin(Eigen::Vector4f(0.25, -4.04, 0.254, 1.0));
        clipper_ground3.setMax(Eigen::Vector4f(0.673, -3.5, 3.0, 1.0));
        clipper_ground3.setInputCloud(temp2.makeShared());
        clipper_ground3.setNegative(true);
        clipper_ground3.filter(temp3);
        temp2.clear();
        //4
        pcl::CropBox<pcl::PointXYZ> clipper_collision4;
        clipper_collision4.setMin(Eigen::Vector4f(-1.9, -1.45, 0.157, 1.0));
        clipper_collision4.setMax(Eigen::Vector4f(-1.38, -0.82, 3.0, 1.0));
        clipper_collision4.setInputCloud(cloud_c);
        clipper_collision4.setNegative(false);
        clipper_collision4.filter(collision4);

        pcl::PointCloud<pcl::PointXYZ> temp4;
        pcl::CropBox<pcl::PointXYZ> clipper_ground4;
        clipper_ground4.setMin(Eigen::Vector4f(-1.9, -1.45, 0.157, 1.0));
        clipper_ground4.setMax(Eigen::Vector4f(-1.38, -0.82, 3.0, 1.0));
        clipper_ground4.setInputCloud(temp3.makeShared());
        clipper_ground4.setNegative(true);
        clipper_ground4.filter(temp4);        
        temp3.clear();
        //5
        pcl::CropBox<pcl::PointXYZ> clipper_collision5;
        clipper_collision5.setMin(Eigen::Vector4f(-3.2, 0.195, 0.542, 1.0));
        clipper_collision5.setMax(Eigen::Vector4f(-2.71, 0.65, 3.0, 1.0));
        clipper_collision5.setInputCloud(cloud_c);
        clipper_collision5.setNegative(false);
        clipper_collision5.filter(collision5);        

        pcl::PointCloud<pcl::PointXYZ> temp5;
        pcl::CropBox<pcl::PointXYZ> clipper_ground5;
        clipper_ground5.setMin(Eigen::Vector4f(-3.2, 0.195, 0.542, 1.0));
        clipper_ground5.setMax(Eigen::Vector4f(-2.71, 0.65, 3.0, 1.0));
        clipper_ground5.setInputCloud(temp4.makeShared());
        clipper_ground5.setNegative(true);
        clipper_ground5.filter(temp5);       
        temp4.clear();
        //6
        pcl::CropBox<pcl::PointXYZ> clipper_collision6;
        clipper_collision6.setMin(Eigen::Vector4f(0.098, -0.96, 0.619, 1.0));
        clipper_collision6.setMax(Eigen::Vector4f(0.52, -0.396, 3.0, 1.0));
        clipper_collision6.setInputCloud(cloud_c);
        clipper_collision6.setNegative(false);
        clipper_collision6.filter(collision6);       

        pcl::PointCloud<pcl::PointXYZ> temp6;
        pcl::CropBox<pcl::PointXYZ> clipper_ground6;
        clipper_ground6.setMin(Eigen::Vector4f(0.098, -0.96, 0.619, 1.0));
        clipper_ground6.setMax(Eigen::Vector4f(0.52, -0.396, 3.0, 1.0));
        clipper_ground6.setInputCloud(temp5.makeShared());
        clipper_ground6.setNegative(true);
        clipper_ground6.filter(temp6);        
        temp5.clear();
        //7
        pcl::CropBox<pcl::PointXYZ> clipper_collision7;
        clipper_collision7.setMin(Eigen::Vector4f(2.857, -1.315, 0.275, 1.0));
        clipper_collision7.setMax(Eigen::Vector4f(3.33, -0.879, 3.0, 1.0));
        clipper_collision7.setInputCloud(cloud_c);
        clipper_collision7.setNegative(false);
        clipper_collision7.filter(collision7);       

        pcl::PointCloud<pcl::PointXYZ> temp7;
        pcl::CropBox<pcl::PointXYZ> clipper_ground7;
        clipper_ground7.setMin(Eigen::Vector4f(2.857, -1.315, 0.275, 1.0));
        clipper_ground7.setMax(Eigen::Vector4f(3.33, -0.879, 3.0, 1.0));
        clipper_ground7.setInputCloud(temp6.makeShared());
        clipper_ground7.setNegative(true);
        clipper_ground7.filter(temp7);        
        temp6.clear();
        //8
        pcl::CropBox<pcl::PointXYZ> clipper_collision8;
        clipper_collision8.setMin(Eigen::Vector4f(1.465, -0.116, 0.136, 1.0));
        clipper_collision8.setMax(Eigen::Vector4f(1.898, 0.398, 3.0, 1.0));
        clipper_collision8.setInputCloud(cloud_c);
        clipper_collision8.setNegative(false);
        clipper_collision8.filter(collision8);        

        pcl::PointCloud<pcl::PointXYZ> temp8;
        pcl::CropBox<pcl::PointXYZ> clipper_ground8;
        clipper_ground8.setMin(Eigen::Vector4f(1.465, -0.116, 0.136, 1.0));
        clipper_ground8.setMax(Eigen::Vector4f(1.898, 0.398, 3.0, 1.0));
        clipper_ground8.setInputCloud(temp7.makeShared());
        clipper_ground8.setNegative(true);
        clipper_ground8.filter(temp8);        
        temp7.clear();
        //9
        pcl::CropBox<pcl::PointXYZ> clipper_collision9;
        clipper_collision9.setMin(Eigen::Vector4f(-3.09, 3.294, 0.666, 1.0));
        clipper_collision9.setMax(Eigen::Vector4f(-2.595, 3.880, 3.0, 1.0));
        clipper_collision9.setInputCloud(cloud_c);
        clipper_collision9.setNegative(false);
        clipper_collision9.filter(collision9);       

        pcl::PointCloud<pcl::PointXYZ> temp9;
        pcl::CropBox<pcl::PointXYZ> clipper_ground9;
        clipper_ground9.setMin(Eigen::Vector4f(-3.09, 3.294, 0.666, 1.0));
        clipper_ground9.setMax(Eigen::Vector4f(-2.595, 3.880, 3.0, 1.0));
        clipper_ground9.setInputCloud(temp8.makeShared());
        clipper_ground9.setNegative(true);
        clipper_ground9.filter(temp9);        
        temp8.clear();
        //10
        pcl::CropBox<pcl::PointXYZ> clipper_collision10;
        clipper_collision10.setMin(Eigen::Vector4f(3.0235, 1.4224, 0.295, 1.0));
        clipper_collision10.setMax(Eigen::Vector4f(3.366, 1.82, 3.0, 1.0));
        clipper_collision10.setInputCloud(cloud_c);
        clipper_collision10.setNegative(false);
        clipper_collision10.filter(collision10);
        

        pcl::CropBox<pcl::PointXYZ> clipper_ground10;
        clipper_ground10.setMin(Eigen::Vector4f(3.0235, 1.4224, 0.295, 1.0));
        clipper_ground10.setMax(Eigen::Vector4f(3.366, 1.82, 3.0, 1.0));
        clipper_ground10.setInputCloud(temp9.makeShared());
        clipper_ground10.setNegative(true);
        clipper_ground10.filter(*cloud_g);
        temp9.clear();

        *world_cloud_collision = collision1 
                                + collision2 + collision3
                                + collision4 + collision5 + collision6
                                + collision7 + collision8 + collision9 + collision10;
        world_cloud_collision->width = world_cloud_collision->points.size();
        world_cloud_collision->height = 1;
        world_cloud_collision->is_dense = true;
        world_cloud_collision->header.frame_id = "world";
        pcl::toROSMsg(*world_cloud_collision, collision_cloud_msg);
        
    }
}