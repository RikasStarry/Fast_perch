#include "uneven_map/uneven_map.h"
namespace uneven_planner
{
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

        rs2.assess(2) = 0.0;

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
        nh.getParam("uneven_map/set_noise", set_noise);
        nh.getParam("uneven_map/iter_num", iter_num);
        nh.getParam("uneven_map/show_type", show_type);
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
        //vis_timer = nh.createTimer(ros::Duration(1.0), &UnevenMap::visCallback, this);

        // size
        map_size[2] = 2.0 * M_PI + 5e-2;
        
        // origin and boundary
        min_boundary = Eigen::Vector3d(0.0,-4.0,-map_size(2) / 2.0);
        max_boundary = Eigen::Vector3d(map_size(0),6.0,map_size(2) / 2.0);
        map_origin = min_boundary;

        // resolution
        xy_resolution_inv = 1.0 / xy_resolution;
        yaw_resolution_inv = 1.0 / yaw_resolution;
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

        // world cloud process
        pcl::PointCloud<pcl::PointXYZ> cloudMapOrigin;
        pcl::PointCloud<pcl::PointXYZ> cloudMapClipper;

        pcl::PCDReader reader;
        reader.read<pcl::PointXYZ>(pcd_file, cloudMapOrigin);

        pcl::CropBox<pcl::PointXYZ> clipper;
        clipper.setMin(Eigen::Vector4f(-0.5, -4.0, -0.1, 1.0));
        clipper.setMax(Eigen::Vector4f(16.0, 6.0, 6.0, 1.0));
        clipper.setInputCloud(cloudMapOrigin.makeShared());
        clipper.filter(cloudMapClipper);
        cloudMapOrigin.clear();

        // pcl::VoxelGrid<pcl::PointXYZ> dwzFilter;
        // dwzFilter.setLeafSize(0.01, 0.01, 0.01);
        // dwzFilter.setInputCloud(cloudMapClipper.makeShared());
        // dwzFilter.filter(*world_cloud);
        // cloudMapClipper.clear();
        world_cloud = cloudMapClipper.makeShared();
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

        pcl::toROSMsg(*world_cloud, origin_cloud_msg);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample_map(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/gnij/Fast-Perching-master/src/uneven_map/maps/downsample.pcd", *downsample_map);
        downsample_map->width = downsample_map->points.size();
        downsample_map->height = 1;
        downsample_map->is_dense = true;
        downsample_map->header.frame_id = "world";
        pcl::toROSMsg(*downsample_map, collision_cloud_msg);

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

                    if(pt_rs2.assess(2)>0.0)
                        continue;

                    if ((show_type == 1 && pt_rs2.assess(0) > max_rho)    \
                    || (show_type ==3 && pt_rs2.assess(2) > max_sparsity) \
                    || (show_type ==4 && pt_c < min_cnormal)              \
                    )
                    {
                        occ_buffer[toAddress(x, y, yaw)] = 1;
                        //occ_r2_buffer[x*voxel_num(1)+y] = 1;
                        continue;
                    }

                    if (show_type == 5 &&   \
                    (  pt_c < min_cnormal || pt_rs2.assess(0) > max_rho || pt_rs2.assess(1) > 5.0/12.0 ))
                    {
                        occ_buffer[toAddress(x, y, yaw)] = 1;
                        //occ_r2_buffer[x*voxel_num(1)+y] = 1;
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

        pcl::toROSMsg(grid_map_filtered, filtered_cloud_msg);

        origin_pub.publish(origin_cloud_msg);
        filtered_pub.publish(filtered_cloud_msg);
        collision_pub.publish(collision_cloud_msg);

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
                            rxs2_z.assess(2) = 0.01;
                            double slope = 0.0;
                            rxs2_z.sigma = 0.0;
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
                        if (points.empty())
                        {
                            RXS2 rxs2_z;
                            rxs2_z.z = world_pos(2);
                            rxs2_z.assess(2) = 0.01;
                            double slope = 0.0;
                            rxs2_z.sigma = 0.0;
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
        
        //origin_pub.publish(origin_cloud_msg);
        //filtered_pub.publish(filtered_cloud_msg);
        //collision_pub.publish(collision_cloud_msg);
    }
}