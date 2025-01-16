#include "uneven_map/uneven_map.h"
namespace uneven_planner
{
    int UnevenMap::lowlim = 0.0;
    int UnevenMap::uplim = 0.0;
    Eigen::Vector4d UnevenMap::lamda = Eigen::Vector4d::Zero();

    RXS2 UnevenMap::filter(Eigen::Vector3d pos, vector<Eigen::Vector3d>& points ,double deltah)
    {
        RXS2 rs2;

        Eigen::Vector3d mean_points = Eigen::Vector3d::Zero();
        for (std::size_t i=0; i<points.size(); i++)
            mean_points+=points[i];

        mean_points /= (double)points.size();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (std::size_t i=0; i<points.size(); i++)
        {
            Eigen::Vector3d v = points[i] - mean_points;
            cov += v * v.transpose();
        }
        cov /= (double)points.size();
        Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
        Eigen::Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();
        Eigen::Matrix3d V = es.pseudoEigenvectors();
        Eigen::MatrixXd::Index evalsMax;
        D.minCoeff(&evalsMax);
        Eigen::Matrix<double, 3, 1> n = V.col(evalsMax);
        n.normalize();
        if (n(2, 0) < 0.0)
            n = -n;
        
        rs2.assess(0) = D(evalsMax) / D.sum() * 3.0;
        if (isnan(rs2.assess(0)))
        {
            rs2.assess(0) = 1.0;
            n = Eigen::Vector3d(1.0, 0.0, 0.0);
        }
        rs2.z = mean_points.z();
        rs2.zb.x() = n(0, 0);
        rs2.zb.y() = n(1, 0);
        //flatness
        if(deltah>=1.0)
        {
            rs2.assess(1) = 1.0;
        }
        else
        {
            rs2.assess(1) = deltah;
        }
        //sparsity
        if(points.size()<lowlim)
        {
            rs2.assess(2) = 1.0;
        }
        else if(points.size()>=lowlim && points.size()<=uplim)
        {
            double prop = double(points.size()-lowlim)/double(uplim-lowlim);
            rs2.assess(2) = 1.0-prop;
        }
        else
        {
            rs2.assess(2) = 0.0;
        }
        double slope = 2.0 * acos(rs2.getC())/M_PI;
        rs2.sigma = lamda.dot(Eigen::Vector4d(rs2.assess(0),rs2.assess(1),rs2.assess(2),slope));
        return rs2;
    }

    Eigen::Matrix3d UnevenMap::skewSym(Eigen::Vector3d vec)
    {
        Eigen::Matrix3d skem_sym;
        skem_sym << 0.0    , -vec(2), vec(1) , \
                    vec(2) , 0.0    , -vec(0), \
                    -vec(1), vec(0) , 0.0       ;
        return skem_sym;
    }

    // zb, yb = (zb x xyaw).normalized(), xb = yb x zb
    // using Sherman-Morrison formula
    double UnevenMap::calYawFromR(Eigen::Matrix3d R)
    {
        Eigen::Vector2d p(R(0, 2), R(1, 2));
        Eigen::Vector2d b(R(0, 0), R(1, 0));
        Eigen::Vector2d x = (Eigen::Matrix2d::Identity()+p*p.transpose()/(1.0-p.squaredNorm()))*b;
        return atan2(x(1), x(0));
    }

    void UnevenMap::normSO2(double& yaw)
    {
        while (yaw < -M_PI)
            yaw += 2*M_PI;
        while (yaw > M_PI)
            yaw -= 2*M_PI;
        return;
    }

    void UnevenMap::init(ros::NodeHandle& nh)
    {
        nh.getParam("uneven_map/show_zbso2", show_zbso2);
        nh.getParam("uneven_map/set_noise", set_noise);
        nh.getParam("uneven_map/iter_num", iter_num);
        nh.getParam("uneven_map/show_type", show_type);
        nh.getParam("uneven_map/maxpt_size", maxpt_size);
        nh.getParam("uneven_map/stddev", stddev);
        nh.getParam("uneven_map/map_size_x", map_size[0]);
        nh.getParam("uneven_map/map_size_y", map_size[1]);
        nh.getParam("uneven_map/ellipsoid_x", ellipsoid_x);
        nh.getParam("uneven_map/ellipsoid_y", ellipsoid_y);
        nh.getParam("uneven_map/ellipsoid_z", ellipsoid_z);
        nh.getParam("uneven_map/xy_resolution", xy_resolution);
        nh.getParam("uneven_map/yaw_resolution", yaw_resolution);
        nh.getParam("uneven_map/min_cnormal", min_cnormal);
        nh.getParam("uneven_map/max_sparsity", max_sparsity);
        nh.getParam("uneven_map/offset", offset);
        nh.getParam("uneven_map/flat_r", flat_r);
        nh.getParam("uneven_map/lamda1", lamda1);
        nh.getParam("uneven_map/lamda2", lamda2);
        nh.getParam("uneven_map/lamda3", lamda3);
        nh.getParam("uneven_map/lamda4", lamda4);
        nh.getParam("uneven_map/max_rho", max_rho);
        nh.getParam("uneven_map/gravity", gravity);
        nh.getParam("uneven_map/mass", mass);
        nh.getParam("uneven_map/map_pcd", pcd_file);
        nh.getParam("uneven_map/map_file", map_file);
        origin_pub = nh.advertise<sensor_msgs::PointCloud2>("/origin_map", 1);
        filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_map", 1);
        collision_pub = nh.advertise<sensor_msgs::PointCloud2>("/collision_map", 1);
        vis_timer = nh.createTimer(ros::Duration(1.0), &UnevenMap::visCallback, this);
        if(show_zbso2)
        {
            zb_pub = nh.advertise<visualization_msgs::MarkerArray>("/zb_map", 1);
            so2_test_pub = nh.advertise<visualization_msgs::MarkerArray>("/so2_map", 1);
            iter_pub = nh.advertise<visualization_msgs::MarkerArray>("/iter_map", 1);
        } 

        // size
        map_size[2] = 2.0 * M_PI + 5e-2;
        
        // origin and boundary
        min_boundary = -map_size / 2.0;
        max_boundary = map_size / 2.0;
        map_origin = min_boundary;

        // resolution
        xy_resolution_inv = 1.0 / xy_resolution;
        yaw_resolution_inv = 1.0 / yaw_resolution;
        lowlim = ceil(0.4*maxpt_size);
        uplim = ceil(min_cnormal*maxpt_size);
        lamda << lamda1,lamda2,lamda3,lamda4;
        // voxel num
        voxel_num(0) = ceil(map_size(0) / xy_resolution);
        voxel_num(1) = ceil(map_size(1) / xy_resolution);
        voxel_num(2) = ceil(map_size(2) / yaw_resolution);

        // idx
        min_idx = Eigen::Vector3i::Zero();
        max_idx = voxel_num - Eigen::Vector3i::Ones();

        // datas
        int buffer_size  = voxel_num(0) * voxel_num(1) * voxel_num(2);
        map_buffer = vector<RXS2>(buffer_size, RXS2());
        c_buffer   = vector<double>(buffer_size, 1.0);
        occ_buffer = vector<char>(buffer_size, 0);
        occ_r2_buffer = vector<char>(getXYNum(), 0);
        world_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        world_cloud_plane.reset(new pcl::PointCloud<pcl::PointXY>());
        world_cloud_collision.reset(new pcl::PointCloud<pcl::PointXYZ>());

        static std::random_device rd;
        static std::mt19937 gen(rd()); 
        static std::normal_distribution<double> noise_x(0.0, stddev); 
        static std::normal_distribution<double> noise_y(0.0, stddev);
        static std::normal_distribution<double> noise_z(0.0, stddev);

        // world cloud process
        pcl::PointCloud<pcl::PointXYZ> cloudMapOrigin_collision;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMapOrigin_ground;
        cloudMapOrigin_ground.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ> cloudMapClipper;

        pcl::PCDReader reader;
        //reader.read<pcl::PointXYZ>(pcd_file, cloudMapOrigin_ground);
        reader.read<pcl::PointXYZ>(pcd_file, cloudMapOrigin_collision);

        cloudMapOrigin_collision.width = cloudMapOrigin_collision.points.size();
        cloudMapOrigin_collision.height = 1;
        cloudMapOrigin_collision.is_dense = true;
        cloudMapOrigin_collision.header.frame_id = "world";

        if(set_noise)
        {
            //带噪声的origin
            pcl::PointCloud<pcl::PointXYZ> cloudMapOrigin_collision_temp = cloudMapOrigin_collision;
            cloudMapOrigin_collision_temp.header.frame_id = "world";
            cloudMapOrigin_collision.width = cloudMapOrigin_collision.points.size();
            cloudMapOrigin_collision.height = 1;
            cloudMapOrigin_collision.is_dense = true;
            for (auto& point : cloudMapOrigin_collision_temp)
            {
                point.x += noise_x(gen);
                point.y += noise_y(gen);
                point.z += noise_z(gen);
            }
            pcl::toROSMsg(cloudMapOrigin_collision_temp, origin_cloud_msg);
        }
        else
        {
            pcl::toROSMsg(cloudMapOrigin_collision, origin_cloud_msg);
        }
        //点云切割
        pointSeg(cloudMapOrigin_collision.makeShared(),cloudMapOrigin_ground);
        cloudMapOrigin_collision.clear();

        pcl::CropBox<pcl::PointXYZ> clipper;
        clipper.setMin(Eigen::Vector4f(-10.0, -10.0, -0.01, 1.0));
        clipper.setMax(Eigen::Vector4f(10.0, 10.0, 5.0, 1.0));
        clipper.setInputCloud(cloudMapOrigin_ground);
        clipper.filter(cloudMapClipper);
        cloudMapOrigin_ground->clear();

        pcl::VoxelGrid<pcl::PointXYZ> dwzFilter;
        dwzFilter.setLeafSize(0.01, 0.01, 0.01);
        dwzFilter.setInputCloud(cloudMapClipper.makeShared());
        dwzFilter.filter(*world_cloud);
        cloudMapClipper.clear();

        for (std::size_t i=0; i<world_cloud->points.size(); i++)
        {
            pcl::PointXY p;
            p.x = world_cloud->points[i].x;
            p.y = world_cloud->points[i].y;
            world_cloud_plane->points.emplace_back(p);
        }
        world_cloud->width = world_cloud->points.size();
        world_cloud->height = 1;
        world_cloud->is_dense = true;
        world_cloud->header.frame_id = "world";
        world_cloud_plane->width = world_cloud_plane->points.size();
        world_cloud_plane->height = 1;
        world_cloud_plane->is_dense = true;
        world_cloud_plane->header.frame_id = "world";
        kd_tree.setInputCloud(world_cloud);
        kd_tree_plane.setInputCloud(world_cloud_plane);
        //pcl::toROSMsg(*world_cloud, origin_cloud_msg);

        // construct map: SO(2) --> RXS2
        if (!constructMapInput())
            constructMap();
        double max_r = 0.0;
        double min_r = max_rho;
        double max_sp = 0.0;
        double min_sp = 1.0;
        double max_sig = 0.0;
        double min_sig = 1.0;

        // occ map
        for (int x=0; x<voxel_num[0]; x++)
            for (int y=0; y<voxel_num[1]; y++)
                for (int yaw=0; yaw<voxel_num[2]; yaw++)
                {   
                    RXS2 pt_rs2 = map_buffer[toAddress(x, y, yaw)] ;
                    double pt_c = c_buffer[toAddress(x, y, yaw)];
                    if ((show_type == 1 && pt_rs2.assess(0) > max_rho)    \
                    || (show_type ==3 && pt_rs2.assess(2) > max_sparsity) \
                    || (show_type ==4 && pt_c < min_cnormal)              \
                    )
                    {
                        occ_buffer[toAddress(x, y, yaw)] = 1;
                        occ_r2_buffer[x*voxel_num(1)+y] = 1;
                        continue;
                    }

                    if (show_type == 5 &&   \
                    (  pt_c < min_cnormal || pt_rs2.assess(0) > max_rho || pt_rs2.assess(2) > max_sparsity ))
                    {
                        occ_buffer[toAddress(x, y, yaw)] = 1;
                        occ_r2_buffer[x*voxel_num(1)+y] = 1;
                    }

                    if(pt_rs2.assess(0) <= max_rho)
                    {
                        if(max_r < pt_rs2.assess(0))
                        {
                            max_r = pt_rs2.assess(0);
                        }
                        if(min_r > pt_rs2.assess(0))
                        {
                            min_r = pt_rs2.assess(0);
                        }
                    }
                    if(pt_rs2.assess(2) <= max_sparsity)
                    {
                        if(max_sp < pt_rs2.assess(2))
                        {
                            max_sp = pt_rs2.assess(2);
                        }
                        if(min_sp > pt_rs2.assess(2))
                        {
                            min_sp = pt_rs2.assess(2);
                        }
                    }
                    if(pt_rs2.sigma <= max_rho)
                    {
                        if(max_sig < pt_rs2.sigma)
                        {
                            max_sig = pt_rs2.sigma;
                        }
                        if(min_sig > pt_rs2.sigma)
                        {
                            min_sig = pt_rs2.sigma;
                        } 
                    } 
                }
                
        std::cout << "max_r:" << max_r << " min_r:" << min_r         \
             << " max_sp:" << max_sp << " min_sp:" << min_sp     \
             << " max_sig:" << max_sig << " min_sig:" << min_sig << std::endl;
        //  to pcl and marker msg
        
        pcl::PointCloud<pcl::PointXYZRGB> grid_map_filtered;
        pcl::PointXYZRGB pt_filtered;
        int yaw = floor(3.0*M_PI_2*yaw_resolution_inv);
        for (int x=0; x<voxel_num[0]; x++)
            for (int y=0; y<voxel_num[1]; y++)
            {
                Eigen::Vector3d filtered_p;
                RXS2 rs2 = map_buffer[toAddress(x, y, yaw)];
                double c = c_buffer[toAddress(x, y, yaw)];
                indexToPos(Eigen::Vector3i(x, y, yaw), filtered_p);

                pt_filtered.x = filtered_p.x();
                pt_filtered.y = filtered_p.y();
                pt_filtered.z = rs2.z;
                if(show_zbso2)
                {
                    //zbso2
                    //if(pt_filtered.z<1.8)
                    //iter
                    if(pt_filtered.x>0 && pt_filtered.x*pt_filtered.x+pt_filtered.y*pt_filtered.y<=6.25 && rs2.z<1.0)
                    {
                        if(rs2.z<0.5 && rs2.z>0.01 && pt_filtered.x>0 && pt_filtered.y/pt_filtered.x<=1.0/3.0 && pt_filtered.y/pt_filtered.x>=-1.0/3.0)
                        {
                            pt_filtered.r = 173;
                            pt_filtered.g = 213;
                            pt_filtered.b = 162;
                            grid_map_filtered.emplace_back(pt_filtered);
                            continue;
                        }
                        else
                        {   
                            pt_filtered.r = pt_blue(0);
                            pt_filtered.g = pt_blue(1);
                            pt_filtered.b = pt_blue(2);
                            grid_map_filtered.emplace_back(pt_filtered);
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }

                if((show_type!=3 && show_type!=5) && map_buffer[toAddress(x, y, yaw)].assess(2)>max_sparsity)
                {
                    continue;
                }
                if(occ_buffer[toAddress(x, y, yaw)]==1)
                {
                    if(show_type!=3 && show_type!=5)
                    {
                            pt_filtered.r = pt_red(0);
                            pt_filtered.g = pt_red(1);
                            pt_filtered.b = pt_red(2);
                    }
                    else
                    {
                        if(map_buffer[toAddress(x, y, yaw)].assess(2)>1.0)
                        {
                            pt_filtered.r = pt_purple(0);
                            pt_filtered.g = pt_purple(1);
                            pt_filtered.b = pt_purple(2);
                        }
                        else
                        {
                            pt_filtered.r = pt_red(0);
                            pt_filtered.g = pt_red(1);
                            pt_filtered.b = pt_red(2);
                        }
                    }
                }
                else if(show_type == 1)
                {
                    if(rs2.assess(0)<=max_rho && rs2.assess(0)>0.5*max_rho )
                    {
                        pt_filtered.r = pt_yellow(0);
                        pt_filtered.g = pt_yellow(1);
                        pt_filtered.b = pt_yellow(2);
                    }
                    else
                    {
                        pt_filtered.r = pt_blue(0);
                        pt_filtered.g = pt_blue(1);
                        pt_filtered.b = pt_blue(2);
                    }
                }
                else if(show_type == 2)
                {
                    if(rs2.assess(1)<=1.0 && rs2.assess(1)>2.0/3.0)
                    {
                        pt_filtered.r = pt_red(0);
                        pt_filtered.g = pt_red(1);
                        pt_filtered.b = pt_red(2);
                    }
                    else if(rs2.assess(1)<=2.0/3.0 && rs2.assess(1)>1.0/3.0)
                    {
                        pt_filtered.r = pt_yellow(0);
                        pt_filtered.g = pt_yellow(1);
                        pt_filtered.b = pt_yellow(2);
                    }
                    else
                    {
                        pt_filtered.r = pt_blue(0);
                        pt_filtered.g = pt_blue(1);
                        pt_filtered.b = pt_blue(2);
                    }
                }
                else if(show_type == 3)
                {
                    if(rs2.assess(2)<=max_sparsity && rs2.assess(2)>0.8*max_sparsity)
                    {
                        pt_filtered.r = pt_yellow(0);
                        pt_filtered.g = pt_yellow(1);
                        pt_filtered.b = pt_yellow(2);
                    }
                    else
                    {
                        pt_filtered.r = pt_blue(0);
                        pt_filtered.g = pt_blue(1);
                        pt_filtered.b = pt_blue(2);
                    }
                }
                else if(show_type ==4)
                {
                    if(c <= 0.5 + 0.5 * min_cnormal)
                    {
                        pt_filtered.r = pt_yellow(0);
                        pt_filtered.g = pt_yellow(1);
                        pt_filtered.b = pt_yellow(2);
                    }
                    else
                    {
                        pt_filtered.r = pt_blue(0);
                        pt_filtered.g = pt_blue(1);
                        pt_filtered.b = pt_blue(2);
                    }
                }
                else if(show_type == 5)
                {
                    if(c <= 0.5 + 0.5 * min_cnormal)//rs2.sigma>0.5*(max_sig+min_sig)
                    {
                        pt_filtered.r = pt_yellow(0);
                        pt_filtered.g = pt_yellow(1);
                        pt_filtered.b = pt_yellow(2);
                    }
                    else
                    {
                        pt_filtered.r = pt_blue(0);
                        pt_filtered.g = pt_blue(1);
                        pt_filtered.b = pt_blue(2);
                    }
                }
                else
                {
                    std::cout<<"The show_type is wrong!"<<std::endl;
                    return ;
                }
                
                grid_map_filtered.emplace_back(pt_filtered);
            }
        grid_map_filtered.width = grid_map_filtered.points.size();
        grid_map_filtered.height = 1;
        grid_map_filtered.is_dense = true;
        grid_map_filtered.header.frame_id = "world";
        if(set_noise)
        {
            for (auto& point : grid_map_filtered)
            {
                point.x += noise_x(gen);
                point.y += noise_y(gen);
                point.z += noise_z(gen);
            }
        }
        pcl::toROSMsg(grid_map_filtered, filtered_cloud_msg);

        if(show_zbso2)
        {
            visualization_msgs::Marker zb_arrow;            
            zb_arrow.type = visualization_msgs::Marker::ARROW;
            zb_arrow.action = visualization_msgs::Marker::ADD;
            zb_arrow.header.frame_id = "world";
            zb_arrow.pose.orientation.w = 1.0;
            zb_arrow.scale.x = 0.025;//杆宽度
            zb_arrow.scale.y = 0.05;//头宽度
            zb_arrow.scale.z = 0.06;//头高度
            zb_arrow.color.a = 1.0;
            int yaw = floor(3.0*M_PI_2*yaw_resolution_inv);
            for (int x=94; x<107; x+=3)
            {
                if(x==voxel_num[0]/2)
                {
                    continue;
                }
                zb_arrow.points.clear();
                geometry_msgs::Point start, end;
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, voxel_num(1)/2, yaw), pos);
                RXS2 rs2 = map_buffer[toAddress(x, voxel_num(1)/2, yaw)];
                double c = c_buffer[toAddress(x, voxel_num(1)/2, yaw)];
                start.x = pos.x();
                start.y = 0;
                start.z = rs2.z;
                end.x = start.x + 5.0 * xy_resolution * rs2.zb.x();
                end.y = start.y + 5.0 * xy_resolution * rs2.zb.y();
                end.z = start.z + 5.0 * xy_resolution * c;

                zb_arrow.id = x;
                zb_arrow.color.r = pt_green(0)/255.0;
                zb_arrow.color.g = pt_green(1)/255.0;
                zb_arrow.color.b = pt_green(2)/255.0;
                zb_arrow.points.push_back(start);
                zb_arrow.points.push_back(end);
                zb_msg.markers.emplace_back(zb_arrow);
            }
            for (int y=94; y<107; y+=3)
            {
                zb_arrow.points.clear();
                geometry_msgs::Point start, end;
                if(y==voxel_num[1]/2)
                {
                    start.x = 0;
                    start.y = 0;
                    start.z = 2;
                    end.x = 0;
                    end.y = 0;
                    end.z = 2 + 5.0 * xy_resolution;

                    zb_arrow.id = y + voxel_num[0];
                    zb_arrow.color.r = pt_purple(0)/255.0;
                    zb_arrow.color.g = pt_purple(1)/255.0;
                    zb_arrow.color.b = pt_purple(2)/255.0;
                    zb_arrow.points.push_back(start);
                    zb_arrow.points.push_back(end);
                    zb_msg.markers.emplace_back(zb_arrow);
                    continue;
                }
                
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(voxel_num(0)/2, y, yaw), pos);
                RXS2 rs2 = map_buffer[toAddress(voxel_num(0)/2, y, yaw)];
                double c = c_buffer[toAddress(voxel_num(0)/2, y, yaw)];
                start.x = 0;
                start.y = pos.y();
                start.z = rs2.z;
                end.x = start.x + 5.0 * xy_resolution * rs2.zb.x();
                end.y = start.y + 5.0 * xy_resolution * rs2.zb.y();
                end.z = start.z + 5.0 * xy_resolution * c;

                zb_arrow.id = y + voxel_num[0];
                zb_arrow.color.r = pt_green(0)/255.0;
                zb_arrow.color.g = pt_green(1)/255.0;
                zb_arrow.color.b = pt_green(2)/255.0;
                zb_arrow.points.push_back(start);
                zb_arrow.points.push_back(end);
                zb_msg.markers.emplace_back(zb_arrow);
            }

            visualization_msgs::Marker so2_arrow;            
            so2_arrow.type = visualization_msgs::Marker::ARROW;
            so2_arrow.action = visualization_msgs::Marker::ADD;
            so2_arrow.header.frame_id = "world";
            so2_arrow.pose.orientation.w = 1.0;
            so2_arrow.scale.x = 0.025;//杆宽度
            so2_arrow.scale.y = 0.05;//头宽度
            so2_arrow.scale.z = 0.06;//头高度
            so2_arrow.color.a = 0.8;

            visualization_msgs::Marker so2_arrow1;            
            so2_arrow1.type = visualization_msgs::Marker::ARROW;
            so2_arrow1.action = visualization_msgs::Marker::ADD;
            so2_arrow1.header.frame_id = "world";
            so2_arrow1.pose.orientation.w = 1.0;
            so2_arrow1.scale.x = 0.025;//杆宽度
            so2_arrow1.scale.y = 0.05;//头宽度
            so2_arrow1.scale.z = 0.06;//头高度
            so2_arrow1.color.a = 0.8;

            visualization_msgs::Marker so2_ellipsoid;            
            so2_ellipsoid.type = visualization_msgs::Marker::SPHERE;
            so2_ellipsoid.action = visualization_msgs::Marker::ADD;
            so2_ellipsoid.header.frame_id = "world";
            so2_ellipsoid.scale.x = ellipsoid_x;
            so2_ellipsoid.scale.y = ellipsoid_y;
            so2_ellipsoid.scale.z = ellipsoid_z;
            so2_ellipsoid.color.a = 0.5;
            so2_ellipsoid.color.r = 1.0;
            so2_ellipsoid.color.g = 0.0;
            so2_ellipsoid.color.b = 0.0;

            visualization_msgs::Marker yaw_arrow;
            yaw_arrow.type = visualization_msgs::Marker::ARROW;
            yaw_arrow.action = visualization_msgs::Marker::ADD;
            yaw_arrow.header.frame_id = "world";
            yaw_arrow.pose.orientation.w = 1.0;
            yaw_arrow.scale.x = 0.003;//杆宽度
            yaw_arrow.scale.y = 0.015;//头宽度
            yaw_arrow.scale.z = 0.018;//头高度
            yaw_arrow.color.a = 1.0;

            visualization_msgs::Marker line;
            line.type = visualization_msgs::Marker::LINE_LIST;
            line.header.frame_id = "world";
            line.action = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.005;
            line.color.a = 1.0;

            for(int yaw=0; yaw<voxel_num[2]; yaw+=8)
            {
                so2_arrow.points.clear();
                so2_arrow1.points.clear();
                yaw_arrow.points.clear();
                line.points.clear();

                geometry_msgs::Point start, end;
                double yawAngle = yaw * yaw_resolution;
                Eigen::AngleAxisd yawRotation(yawAngle, Eigen::Vector3d::UnitZ());
                Eigen::Matrix3d Ryaw(yawRotation);
                Eigen::AngleAxisd pitchRotation(7.0*M_PI/50.0, Ryaw.col(1));
                Eigen::Matrix3d R(pitchRotation * yawRotation);
                Eigen::Vector3d zb = R.col(2);
                Eigen::Vector3d xb = R.col(0);
                start.x = 5.0 * xy_resolution * cos(yawAngle);
                start.y = 5.0 * xy_resolution * sin(yawAngle);
                start.z = 2.1-0.12-0.095;
                end.x = start.x + 3.0 * xy_resolution * zb.x();
                end.y = start.y + 3.0 * xy_resolution * zb.y();
                end.z = start.z + 3.0 * xy_resolution * zb.z();
                Eigen::Quaterniond q(R);
                q.normalize();
                geometry_msgs::Point end1;
                end1.x = start.x + 3.5 * xy_resolution * xb.x();
                end1.y = start.y + 3.5 * xy_resolution * xb.y();
                end1.z = start.z + 3.5 * xy_resolution * xb.z();

                so2_ellipsoid.id = yaw;
                so2_ellipsoid.pose.position.x = start.x; 
                so2_ellipsoid.pose.position.y = start.y;
                so2_ellipsoid.pose.position.z = start.z;
                so2_ellipsoid.pose.orientation.w = q.w();
                so2_ellipsoid.pose.orientation.x = q.x();
                so2_ellipsoid.pose.orientation.y = q.y();
                so2_ellipsoid.pose.orientation.z = q.z();
                so2_test_msg.markers.emplace_back(so2_ellipsoid);

                so2_arrow.id = yaw + 1;
                so2_arrow.color.r = 192.0/255.0;
                so2_arrow.color.g = 196.0/255.0;
                so2_arrow.color.b = 195.0/255.0;
                so2_arrow.points.push_back(start);
                so2_arrow.points.push_back(end);
                so2_test_msg.markers.emplace_back(so2_arrow);

                so2_arrow1.id = yaw + 2;
                so2_arrow1.color.r = so2_arrow.color.r;
                so2_arrow1.color.g = so2_arrow.color.g;
                so2_arrow1.color.b = so2_arrow.color.b;
                so2_arrow1.points.push_back(start);
                so2_arrow1.points.push_back(end1);
                so2_test_msg.markers.emplace_back(so2_arrow1);

                geometry_msgs::Point start2, end2;
                start2.x = 0;
                start2.y = 0;
                start2.z = 2.0;
                end2.x = start2.x + 2.0 * xy_resolution * cos(yawAngle);
                end2.y = start2.y + 2.0 * xy_resolution * sin(yawAngle);
                end2.z = start2.z;
                yaw_arrow.id = yaw + 3;
                yaw_arrow.color.r = so2_arrow.color.r;
                yaw_arrow.color.g = so2_arrow.color.g;
                yaw_arrow.color.b = so2_arrow.color.b; 
                yaw_arrow.points.push_back(start2);
                yaw_arrow.points.push_back(end2);
                so2_test_msg.markers.emplace_back(yaw_arrow);

                line.id = yaw + 4;
                line.color.r = so2_arrow.color.r;
                line.color.g = so2_arrow.color.g;
                line.color.b = so2_arrow.color.b;
                geometry_msgs::Point midpt;
                for(int seg=0;seg<14;++seg)
                {
                    midpt.x = double(seg)/13.0*5.0 * xy_resolution * cos(yawAngle);
                    midpt.y = double(seg)/13.0*5.0 * xy_resolution * sin(yawAngle);
                    midpt.z = 2.0+double(seg)/13.0*(start.z-2.0);
                    line.points.push_back(midpt);
                } 
                so2_test_msg.markers.emplace_back(line);  
            }
            
            Eigen::Vector3d pos;
            Eigen::Matrix3d R;
            //getTerrainPos(Eigen::Vector3d(1.7,0,0),R,pos);
            indexToPos(Eigen::Vector3i(132, 100, 0), pos);
            visualization_msgs::Marker iter_ellipsoid;            
            iter_ellipsoid.type = visualization_msgs::Marker::SPHERE;
            iter_ellipsoid.action = visualization_msgs::Marker::ADD;
            iter_ellipsoid.header.frame_id = "world";
            iter_ellipsoid.scale.x = 2 * ellipsoid_x;
            iter_ellipsoid.scale.y = 2 * ellipsoid_y;
            iter_ellipsoid.scale.z = 2 * ellipsoid_z;
            iter_ellipsoid.color.a = 0.5;
            iter_ellipsoid.color.r = 1.0;
            iter_ellipsoid.color.g = 0.0;
            iter_ellipsoid.color.b = 0.0;
            iter_ellipsoid.id = 0;
            iter_ellipsoid.pose.position.x = pos.x(); 
            iter_ellipsoid.pose.position.y = pos.y();
            iter_ellipsoid.pose.position.z = map_buffer[toAddress(132, 100, 0)].z;
            Eigen::Vector3d pos1;
            indexToPos(Eigen::Vector3i(133, 100, 0), pos1);
            double pitch =atan2((pos1.x()-pos.x()),(map_buffer[toAddress(132, 100, 0)].z-map_buffer[toAddress(133, 100, 0)].z));
            //double pitch = M_PI/6.0;
            iter_ellipsoid.pose.orientation.w = cos(pitch/2.0);
            iter_ellipsoid.pose.orientation.x = 0;
            iter_ellipsoid.pose.orientation.y = sqrt(1.0-pow(iter_ellipsoid.pose.orientation.w,2));
            iter_ellipsoid.pose.orientation.z = 0;
            iter_msg.markers.emplace_back(iter_ellipsoid); 

            visualization_msgs::Marker iter_arrow;            
            iter_arrow.type = visualization_msgs::Marker::ARROW;
            iter_arrow.action = visualization_msgs::Marker::ADD;
            iter_arrow.header.frame_id = "world";
            iter_arrow.pose.orientation.w = 1.0;
            iter_arrow.scale.x = 0.015;//杆宽度
            iter_arrow.scale.y = 0.03;//头宽度
            iter_arrow.scale.z = 0.04;//头高度
            iter_arrow.color.a = 1.0;

            visualization_msgs::Marker iter_arrow1;            
            iter_arrow1.type = visualization_msgs::Marker::ARROW;
            iter_arrow1.action = visualization_msgs::Marker::ADD;
            iter_arrow1.header.frame_id = "world";
            iter_arrow1.pose.orientation.w = 1.0;
            iter_arrow1.scale.x = 0.015;//杆宽度
            iter_arrow1.scale.y = 0.03;//头宽度
            iter_arrow1.scale.z = 0.04;//头高度
            iter_arrow1.color.a = 1.0;

            geometry_msgs::Point start3,end3;
            start3.x = pos.x();
            start3.y = pos.y();
            start3.z = iter_ellipsoid.pose.position.z;
            end3.x = pos.x() + cos(pitch) * 0.25;
            end3.y = pos.y();
            end3.z = iter_ellipsoid.pose.position.z - sin(pitch) * 0.25;
            iter_arrow.id = 1;
            iter_arrow.color.r = 237.0/255.0;
            iter_arrow.color.g = 227.0/255.0;
            iter_arrow.color.b = 231.0/255.0;
            iter_arrow.points.push_back(start3);
            iter_arrow.points.push_back(end3);
            iter_msg.markers.emplace_back(iter_arrow);

            geometry_msgs::Point end4;
            end4.x = pos.x() + sin(pitch) * 0.15;
            end4.y = pos.y();
            end4.z = iter_ellipsoid.pose.position.z + cos(pitch) * 0.15;
            iter_arrow1.id = 2;
            iter_arrow1.color.r = iter_arrow.color.r;
            iter_arrow1.color.g = iter_arrow.color.g;
            iter_arrow1.color.b = iter_arrow.color.b;
            iter_arrow1.points.push_back(start3);
            iter_arrow1.points.push_back(end4);
            iter_msg.markers.emplace_back(iter_arrow1); 

            visualization_msgs::Marker plane;
            plane.type = visualization_msgs::Marker::LINE_STRIP;
            plane.header.frame_id = "world";
            plane.action = visualization_msgs::Marker::ADD;
            plane.pose.orientation.w = 1.0;
            plane.scale.x = 0.005;
            plane.color.a = 1.0;
            plane.id = 3;
            plane.color.r = pt_yellow(0)/255.0;
            plane.color.g = pt_yellow(1)/255.0;
            plane.color.b = pt_yellow(2)/255.0;
            geometry_msgs::Point corner;
            corner.x = pos.x() - 0.25 * cos(pitch);
            corner.y = pos.y() - 0.15;
            corner.z = iter_ellipsoid.pose.position.z + 0.25 * sin(pitch);
            plane.points.push_back(corner); 
            corner.x = pos.x() + 0.25 * cos(pitch);
            corner.y = pos.y() - 0.15;
            corner.z = iter_ellipsoid.pose.position.z - 0.25 * sin(pitch);
            plane.points.push_back(corner);
            corner.x = pos.x() + 0.25 * cos(pitch);
            corner.y = pos.y() + 0.15;
            corner.z = iter_ellipsoid.pose.position.z - 0.25 * sin(pitch);
            plane.points.push_back(corner);
            corner.x = pos.x() - 0.25 * cos(pitch);
            corner.y = pos.y() + 0.15;
            corner.z = iter_ellipsoid.pose.position.z + 0.25 * sin(pitch);
            plane.points.push_back(corner);
            corner.x = pos.x() - 0.25 * cos(pitch);
            corner.y = pos.y() - 0.15;
            corner.z = iter_ellipsoid.pose.position.z + 0.25 * sin(pitch);
            plane.points.push_back(corner); 
            iter_msg.markers.emplace_back(plane); 
        }
        map_ready = true;
    }

    bool UnevenMap::constructMapInput()
    {
        ifstream pp(map_file);
        if (!pp.good())
        {
            ROS_WARN("map file is empty, begin construct it.");
            return false;
        }
        ifstream fp;
        fp.open(map_file, ios::in);
        string idata, word;
        istringstream sin;
        vector<string> words;
        while (getline(fp, idata))
        {
            sin.clear();
            sin.str(idata);
            words.clear();
            while (getline(sin, word, ','))
            {
                words.emplace_back(word);
            }

            int x = atoi(words[0].c_str());
            int y = atoi(words[1].c_str());
            int yaw = atoi(words[2].c_str());
            double z = stold(words[3]);
            double sigma = stold(words[4]);
            double zba = stold(words[5]);
            double zbb = stold(words[6]);
            double rough = stold(words[7]);
            double flat = stold(words[8]);
            double spar = stold(words[9]);
            if (isInMap(Eigen::Vector3i(x, y, yaw)))
            {
                map_buffer[toAddress(x, y, yaw)] = RXS2(z, sigma, Eigen::Vector2d(zba, zbb),Eigen::Vector3d(rough,flat,spar));
                if (map_buffer[toAddress(x, y, yaw)].sigma < sigma)
                {
                    map_buffer[toAddress(x, y, yaw)].sigma = sigma;
                }
                c_buffer[toAddress(x, y, yaw)] = sqrt(1.0-zba*zba-zbb*zbb);
            }
        }
        fp.close();

        ROS_INFO("map: SO(2) --> RXS2 done.");

        return true;
    }

    bool UnevenMap::constructMap()
    {
        ros::Time start_time = ros::Time::now();
        const double box_r = max(max(ellipsoid_x, ellipsoid_y), ellipsoid_z);
        const Eigen::Vector3d ellipsoid_vecinv(1.0 / ellipsoid_x, 1.0 / ellipsoid_y, 1.0 / ellipsoid_z);
        int cnt=0;
        double minpt_size = 10000;
        double maxpt_size =0;

        for (int x=0; x<voxel_num[0]; x++)
            for (int y=0; y<voxel_num[1]; y++)
                for (int yaw=0; yaw<voxel_num[2]; yaw++)
                    for (int iter=0; iter<iter_num; iter++)
                    {
                        Eigen::Vector3d map_pos;
                        RXS2 map_rs2 = map_buffer[toAddress(x, y, yaw)];
                        double map_c = c_buffer[toAddress(x, y, yaw)];
                        indexToPos(Eigen::Vector3i(x, y, yaw), map_pos);
                        
                        Eigen::Vector3d xyaw(cos(map_pos(2)), sin(map_pos(2)), 0.0);
                        Eigen::Vector3d zb(map_rs2.zb(0), map_rs2.zb(1), map_c);
                        Eigen::Vector3d yb = zb.cross(xyaw).normalized();
                        Eigen::Vector3d xb = yb.cross(zb);
                        Eigen::Matrix3d RT;
                        RT.row(0) = xb;
                        RT.row(1) = yb;
                        RT.row(2) = zb;
                        Eigen::Vector3d world_pos(map_pos(0), map_pos(1), map_rs2.z);
                        world_pos.head(2) += xb.head(2) * offset;
                        //if(yaw == 16 && cnt%100000 == 0)
                        //{
                        //   std::cout<<xb.transpose()<<std::endl;
                        //}
                        
                        vector<int> Idxs;
                        vector<float> SquaredDists;
                        if (iter == 0)
                        {
                            pcl::PointXY pxy;
                            pxy.x = world_pos(0);
                            pxy.y = world_pos(1);
                            if (kd_tree_plane.nearestKSearch(pxy, 1, Idxs, SquaredDists) > 0)
                            {
                                world_pos(2) = world_cloud->points[Idxs[0]].z;
                            }
                        }

                        // get points and compute, update
                        vector<Eigen::Vector3d> points;
                        double minh = 0.0;
                        double maxh = 0.0;
                        pcl::PointXYZ pt;
                        pt.x = world_pos(0);
                        pt.y = world_pos(1);
                        pt.z = world_pos(2);

                        if (!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z))
                        {
                            RXS2 rxs2_z;
                            rxs2_z.z = 0;
                            rxs2_z.assess(2) = 1.001;
                            double slope = 0.0;
                            rxs2_z.sigma = lamda.dot(Eigen::Vector4d(rxs2_z.assess(0),rxs2_z.assess(1),rxs2_z.assess(2),slope));
                            map_buffer[toAddress(x, y, yaw)] = rxs2_z;
                            c_buffer[toAddress(x, y, yaw)] = map_buffer[toAddress(x, y, yaw)].getC();
                            continue;
                        }
                    
                        if (kd_tree.radiusSearch(pt, box_r, Idxs, SquaredDists) > 0)
                        {
                            // is in ellipsoid
                            for (std::size_t i=0; i<Idxs.size(); i++)
                            {
                                Eigen::Vector3d temp_pos(world_cloud->points[Idxs[i]].x, \
                                                         world_cloud->points[Idxs[i]].y, \
                                                         world_cloud->points[Idxs[i]].z );
                                Eigen::Vector3d temp_subtract = temp_pos - world_pos;
                                Eigen::Vector3d temp_inrob = RT*temp_subtract;
                                if (ellipsoid_vecinv.cwiseProduct(temp_inrob).squaredNorm() < 1.0)
                                {
                                    points.emplace_back(temp_pos);
                                    if (temp_inrob.cwiseProduct(Eigen::Vector3d::Constant(1.0/flat_r)).squaredNorm() < 1.0)
                                    {
                                        minh = (temp_inrob(2) <= minh) ? temp_inrob(2) : minh;
                                        maxh = (temp_inrob(2) >= maxh) ? temp_inrob(2) : minh;
                                    }
                                }
                            }
                        }
                        if(iter==1)
                        {
                            minpt_size = (points.size()<=minpt_size) ? points.size() : minpt_size;
                            maxpt_size = (points.size()>=maxpt_size) ? points.size() : maxpt_size;
                        }
                        if (points.empty())
                        {
                            RXS2 rxs2_z;
                            rxs2_z.z = world_pos(2);
                            rxs2_z.assess(2) = 1.001;
                            double slope = 2.0 * acos(rxs2_z.getC())/M_PI;
                            rxs2_z.sigma = lamda.dot(Eigen::Vector4d(rxs2_z.assess(0),rxs2_z.assess(1),rxs2_z.assess(2),slope));
                            map_buffer[toAddress(x, y, yaw)] = rxs2_z;
                            c_buffer[toAddress(x, y, yaw)] = map_buffer[toAddress(x, y, yaw)].getC();
                        }
                        else
                        {
                            map_buffer[toAddress(x, y, yaw)] = UnevenMap::filter(map_pos, points,abs(maxh-minh)/ellipsoid_z*2);
                            c_buffer[toAddress(x, y, yaw)] = map_buffer[toAddress(x, y, yaw)].getC();
                            if (x==0||y==0||x==(voxel_num(0)-1)||y==(voxel_num(1)-1))
                            {
                                double new_spar = max_sparsity - 0.01;
                                map_buffer[toAddress(x, y, yaw)].sigma += lamda3 * (new_spar - map_buffer[toAddress(x, y, yaw)].assess(2));
                                map_buffer[toAddress(x, y, yaw)].assess(2) = new_spar;
                            }
                        }
                        if (iter==0 && cnt++ % 100000 == 0)
                        {
                            std::cout<<"\033[1;33m map process "<<toAddress(x, y, yaw)*100.0 / (voxel_num[0]*voxel_num[1]*voxel_num[2])<<"%\033[0m"<<std::endl;
                            cnt=1;
                        }
                    }
        std::cout << "maxpt_size:" << maxpt_size << " minpt_size:" << minpt_size << std::endl;          
        
        // to txt
        ofstream outf;
        outf.open(map_file, ofstream::out);//覆盖写入
        outf.clear();
        for (int x=0; x<voxel_num[0]; x++)
            for (int y=0; y<voxel_num[1]; y++)
                for (int yaw=0; yaw<voxel_num[2]; yaw++)
                {
                    RXS2 rs2 = map_buffer[toAddress(x, y, yaw)];
                    outf << x << "," << y << "," << yaw << "," << rs2.z << "," << rs2.sigma << "," \
                         << rs2.zb.x() << "," << rs2.zb.y() << "," << rs2.assess(0) << ","         \
                         << rs2.assess(1) <<"," << rs2.assess(2) << endl;
                }
        outf.close();

        ros::Time end_time = ros::Time::now();
        ros::Duration time_diff = end_time - start_time;
        ROS_INFO("Time of constructMap is: %f seconds", time_diff.toSec());
        ROS_INFO("map: SE(2) --> RXS2 done.");

        return true;
    }

    void UnevenMap::visCallback(const ros::TimerEvent& /*event*/)
    {
        if (!map_ready)
            return;
        
        origin_pub.publish(origin_cloud_msg);
        filtered_pub.publish(filtered_cloud_msg);
        collision_pub.publish(collision_cloud_msg);
        if(show_zbso2)
        {
            zb_pub.publish(zb_msg);
            so2_test_pub.publish(so2_test_msg);
            iter_pub.publish(iter_msg);
        }
    }
}