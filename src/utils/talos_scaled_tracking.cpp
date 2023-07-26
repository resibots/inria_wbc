#include "inria_wbc/utils/talos_scaled_tracking.hpp"

static const std::string red = "\x1B[31m";
static const std::string rst = "\x1B[0m";
static const std::string bold = "\x1B[1m";

void initialize_vive(inria::ViveTracking& vive){
    //wait for the tracking system to be initialized
        auto it1 = vive.get().find("LHR-FC2F90A4");
        auto it2 = vive.get().find("LHR-21C1BC92"); 
        std::cout << "waiting for vive's initialization process to complete... " << std::endl;
        while (it1 == vive.get().end() || it2 == vive.get().end()){
            vive.update();
            it1 = vive.get().find("LHR-FC2F90A4");
            it2 = vive.get().find("LHR-21C1BC92");
            
        }

        std::cout << "vive initialized successfully, waiting for a valid position... " << std::endl;
        
        //waiting to get the first valid position
        std::cout << "waiting for a valid position...\n" << std::endl;
        while(!vive.get().at("LHR-FC2F90A4").isValid || !vive.get().at("LHR-21C1BC92").isValid){vive.update();}

        std::cout << "valid position found, initializing the tracking simulation process... \n" << std::endl;
}

void draw_ref(const Eigen::Vector3d& center,const Eigen::Matrix3d& rotation,
            const Eigen::Vector4d& color,std::vector<std::shared_ptr<robot_dart::Robot>>& s_list){
    Eigen::Isometry3d iso_center = Eigen::Isometry3d(Eigen::Translation3d(center(0),center(1),center(2)));
    Eigen::Vector3d cx = center + rotation*(Eigen::Vector3d(0.15,0,0));
    Eigen::Vector3d cy = center + rotation*(Eigen::Vector3d(0,0.15,0));
    Eigen::Vector3d cz = center + rotation*(Eigen::Vector3d(0,0,0.15));
    Eigen::Isometry3d iso_cx = Eigen::Isometry3d(Eigen::Translation3d(cx(0),cx(1),cx(2)));
    Eigen::Isometry3d iso_cy = Eigen::Isometry3d(Eigen::Translation3d(cy(0),cy(1),cy(2)));
    Eigen::Isometry3d iso_cz = Eigen::Isometry3d(Eigen::Translation3d(cz(0),cz(1),cz(2)));

    auto s_center = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(0.15,0.15,0.15),iso_center,"fixed",1,color);
    auto s_cx = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(0.1,0.1,0.1),iso_cx,"fixed",1,Eigen::Vector4d(1,0,0,0.5));
    auto s_cy = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(0.1,0.1,0.1),iso_cy,"fixed",1,Eigen::Vector4d(0,1,0,0.5));
    auto s_cz = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(0.1,0.1,0.1),iso_cz,"fixed",1,Eigen::Vector4d(0,0,1,0.5));

    s_center->set_color_mode("aspect");
    s_cx->set_color_mode("aspect");
    s_cy->set_color_mode("aspect");
    s_cz->set_color_mode("aspect");

    s_list.push_back(s_center);
    s_list.push_back(s_cx);
    s_list.push_back(s_cy);
    s_list.push_back(s_cz);
}

void update_ref(const Eigen::Vector3d& center,const Eigen::Matrix3d& rotation,
                std::vector<std::shared_ptr<robot_dart::Robot>>& s_list){

    Eigen::Isometry3d iso_center = Eigen::Isometry3d(Eigen::Translation3d(center(0),center(1),center(2)));
    Eigen::Vector3d cx = center + rotation*(Eigen::Vector3d(0.15,0,0));
    Eigen::Vector3d cy = center + rotation*(Eigen::Vector3d(0,0.15,0));
    Eigen::Vector3d cz = center + rotation*(Eigen::Vector3d(0,0,0.15));
    Eigen::Isometry3d iso_cx = Eigen::Isometry3d(Eigen::Translation3d(cx(0),cx(1),cx(2)));
    Eigen::Isometry3d iso_cy = Eigen::Isometry3d(Eigen::Translation3d(cy(0),cy(1),cy(2)));
    Eigen::Isometry3d iso_cz = Eigen::Isometry3d(Eigen::Translation3d(cz(0),cz(1),cz(2)));

    s_list[0]->set_base_pose(iso_center);
    s_list[1]->set_base_pose(iso_cx);
    s_list[2]->set_base_pose(iso_cy);
    s_list[3]->set_base_pose(iso_cz);
}

Eigen::Vector3d MatrixToEulerIntrinsic(const Eigen::Matrix3d& mat)
{
    //Eigen euler angles and with better range)
    Eigen::Vector3d angles;
    //Roll
    angles.x() = std::atan2(mat(2, 1), mat(2, 2));
    //Pitch
    angles.y() = std::atan2(-mat(2, 0), 
        sqrt(mat(0, 0)*mat(0, 0) 
            + mat(1, 0)*mat(1, 0)));
    //Yaw
    angles.z() = std::atan2(mat(1, 0), mat(0, 0));

    return angles;
}

Eigen::Matrix3d get_trans(const std::shared_ptr<inria_wbc::behaviors::humanoid::FollowTrackers>& behavior,
                            const inria::ViveTracking& vive,const std::string vive_controller){
    if (vive_controller == "LHR-FC2F90A4")
        return behavior->get_init_rot_right()*vive.get().at(vive_controller).matHand.transpose();
    else if (vive_controller == "LHR-21C1BC92")
        return behavior->get_init_rot_left()*vive.get().at(vive_controller).matHand.transpose();
    else
        return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d get_rot_ref(const inria::ViveTracking& vive,const std::string vive_controller){
    double yaw = (double) MatrixToEulerIntrinsic(vive.get().at(vive_controller).matHand).z();
    return Eigen::AngleAxisd(-yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Vector3d get_init_offset(const Eigen::Vector3d hand_init_pos,
                                const inria::ViveTracking& vive,const std::string vive_controller,
                                const Eigen::Matrix3d K)
{
    return K*get_rot_ref(vive,vive_controller)*vive.get().at(vive_controller).posHand - hand_init_pos;
}

Eigen::Vector3d vive_pos_processing(const inria::ViveTracking& vive,const std::string vive_controller,const Eigen::Vector3d init_offset,
                                    const Eigen::Matrix3d rot_ref,const Eigen::Matrix3d K)
{
    return K*rot_ref*vive.get().at(vive_controller).posHand - init_offset;
}

Eigen::Matrix3d vive_rot_processing(const inria::ViveTracking& vive,const std::string vive_controller,
                                    const Eigen::Matrix3d trans)
{
    return trans*vive.get().at(vive_controller).matHand;
}

void draw_obj(std::vector<std::shared_ptr<robot_dart::Robot>>& s_obj_list,
                const std::vector<std::vector<Eigen::Vector3d>>& exercises,const int index)
{
    Eigen::Isometry3d obj_right = Eigen::Isometry3d(Eigen::Translation3d(exercises[0][index].coeff(0),exercises[0][index].coeff(1),exercises[0][index].coeff(2)));
    Eigen::Isometry3d obj_left = Eigen::Isometry3d(Eigen::Translation3d(exercises[1][index].coeff(0),exercises[1][index].coeff(1),exercises[1][index].coeff(2)));
    auto s_obj_right = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(0.2,0.2,0.2),obj_right,"fixed",1,Eigen::Vector4d(1,1,0,1));
    auto s_obj_left = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(0.2,0.2,0.2),obj_left,"fixed",1,Eigen::Vector4d(1,1,0,1));

    s_obj_list.push_back(s_obj_right);
    s_obj_list.push_back(s_obj_left);
}

void update_obj(std::vector<std::shared_ptr<robot_dart::Robot>>& s_obj_list,
                const std::vector<std::vector<Eigen::Vector3d>>& exercises,const int index)
{
    Eigen::Isometry3d obj_right = Eigen::Isometry3d(Eigen::Translation3d(exercises[0][index].coeff(0),exercises[0][index].coeff(1),exercises[0][index].coeff(2)));
    Eigen::Isometry3d obj_left = Eigen::Isometry3d(Eigen::Translation3d(exercises[1][index].coeff(0),exercises[1][index].coeff(1),exercises[1][index].coeff(2)));

    s_obj_list[0]->set_base_pose(obj_right);
    s_obj_list[1]->set_base_pose(obj_left);
}

bool is_obj_achieved(Eigen::Vector3d obj,Eigen::Vector3d current_pos,double epsilon){
    return (obj - current_pos).norm() < epsilon;
}

std::vector<std::shared_ptr<robot_dart::Robot>> draw_dist_between_two_pos(const Eigen::Vector3d pos1,
                                                                    const Eigen::Vector3d pos2,const int nb_pts)
{
    Eigen::Vector3d dist;
    std::vector<std::shared_ptr<robot_dart::Robot>> spheres_to_cover_dist;
    Eigen::Isometry3d current_pos;

    dist = pos2 - pos1;
    for (int i = 0;i < nb_pts;i++){
        Eigen::Vector3d temp = pos1 + i*dist/nb_pts;
        current_pos = Eigen::Isometry3d(Eigen::Translation3d(temp.x(),temp.y(),temp.z()));
        std::shared_ptr<robot_dart::Robot> sphere = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(0.02,0.02,0.02),current_pos,"fixed",1,Eigen::Vector4d(0,1,1,0.5));
        spheres_to_cover_dist.push_back(sphere);
    }
    return spheres_to_cover_dist;
}

void update_dist_between_two_pos(Eigen::Vector3d pos1,Eigen::Vector3d pos2,
                                std::vector<std::shared_ptr<robot_dart::Robot>>& s_to_cov_dist)
{
    Eigen::Vector3d dist = pos2 - pos1;
    Eigen::Isometry3d current_pos;
    int nb_pts = s_to_cov_dist.size();

    for (int i = 0;i < nb_pts;i++){
        Eigen::Vector3d temp = pos1 + i*dist/nb_pts;
        current_pos = Eigen::Isometry3d(Eigen::Translation3d(temp.x(),temp.y(),temp.z()));
        s_to_cov_dist[i]->set_base_pose(current_pos);
    }
}

double calculate_dist_error_by_dir(const Eigen::Vector3d pt1,const Eigen::Vector3d pt2){
    return (pt1 - pt2).lpNorm<1>();
}

double note(double time,int number_of_penalties,double dist_error){
    return time + 10*number_of_penalties + 10*dist_error;
}

double talos_scaled_tracking(int argc,char* argv[],const Eigen::Matrix3d K)
{
    try {
        // program options
        namespace po = boost::program_options;
        po::options_description desc("Test_controller options");
        // clang-format off
        desc.add_options()
        ("actuators,a", po::value<std::string>()->default_value("spd"), "actuator model torque/velocity/servo/spd  [default:spd]")
        ("behavior,b", po::value<std::string>()->default_value("../etc/talos/squat.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/talos/talos_squat.yaml]")
        ("big_window,w", "use a big window (nicer but slower) [default:false]")
        ("check_self_collisions", "check the self collisions (print if a collision)")
        ("check_fall", "check if the robot has fallen (print if a collision)")
        ("collision,k", po::value<std::string>()->default_value("fcl"), "collision engine [default:fcl]")
        ("collisions", po::value<std::string>(), "display the collision shapes for task [name]")
        ("controller,c", po::value<std::string>()->default_value("../etc/talos/talos_pos_tracker.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/talos/talos_pos_tracker.yaml]")
        ("cut", po::value<std::string>()->default_value("leg_left_1_link"), "joint to cut it damage option is enabled")
        ("damage", po::value<bool>()->default_value(false), "damage talos")
        ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
        ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
        ("fast,f", "fast (simplified) Talos [default: false]")
        ("control_freq", po::value<int>()->default_value(1000), "set the control frequency")
        ("sim_freq", po::value<int>()->default_value(1000), "set the simulation frequency")
        ("srdf,s", po::value<float>()->default_value(0.0), "save the configuration at the specified time")
        ("ghost,g", "display the ghost (Pinocchio model)")
        ("model_collisions",po::value<bool>()->default_value(false), "display pinocchio qp model collision spheres")
        ("closed_loop", "Close the loop with floating base position and joint positions; required for torque control [default: from YAML file]")
        ("help,h", "produce help message")
        ("height", po::value<bool>()->default_value(false), "print total feet force data to adjust height in config")
        ("mp4,m", po::value<std::string>(), "save the display to a mp4 video [filename]")
        ("push,p", po::value<std::vector<float>>(), "push the robot at t=x1 0.25 s")
        ("norm_force,n", po::value<float>()->default_value(-150) , "push norm force value")
        ("verbose,v", "verbose mode (controller)")
        ("save_traj,S", po::value<std::vector<std::string>>()->multitoken(), "save the trajectory in dir <dir> for references <refs>: -S traj1 rh lh com")
        ("log,l", po::value<std::vector<std::string>>()->default_value(std::vector<std::string>(),""), 
            "log the trajectory of a dart body [with urdf names] or timing or CoM or cost, example: -l timing -l com -l lf -l cost_com -l cost_lf")
        ;
        
        po::variables_map vm;

        try
        {
            po::store(po::parse_command_line(argc, argv, desc), vm);
            po::notify(vm);
        }
        catch (po::too_many_positional_options_error &e)
        {
            // A positional argument like `opt2=option_value_2` was given
            std::cerr << e.what() << std::endl;
            std::cerr << desc << std::endl;
            return 1;
        }
        catch (po::error_with_option_name &e)
        {
            // Another usage error occurred
            std::cerr << e.what() << std::endl;
            std::cerr << desc << std::endl;
            return 1;
        }

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        // clang-format off
        std::cout<< "------ CONFIGURATION ------" << std::endl;
        std::ostringstream oss_conf;
        for (const auto& kv : vm){
            oss_conf << kv.first << " ";
            try { oss_conf << kv.second.as<std::string>();
            } catch(...) {/* do nothing */ }
            try { oss_conf << kv.second.as<bool>();
            } catch(...) {/* do nothing */ }
            try { oss_conf << kv.second.as<int>();
            } catch(...) {/* do nothing */ }
            oss_conf << std::endl;
        }
        std::cout << oss_conf.str();
        std::cout << "--------------------------" << std::endl;
        // clang-format on

        bool verbose = (vm.count("verbose") != 0);
        std::map<std::string, std::shared_ptr<std::ofstream>> log_files;
        for (auto& x : vm["log"].as<std::vector<std::string>>())
            log_files[x] = std::make_shared<std::ofstream>((x + ".dat").c_str());

        // dt of the simulation and the controller
        int sim_freq = vm["sim_freq"].as<int>();
        float dt = 1.0f / sim_freq;
        std::cout << "dt:" << dt << std::endl;

        //////////////////// INIT DART ROBOT //////////////////////////////////////
        std::srand(std::time(NULL));
        // std::vector<std::pair<std::string, std::string>> packages = {{"talos_data", "/home/pal/talos_data"}};
        // std::string urdf = vm.count("fast") ? "talos/talos_fast.urdf" : "talos/talos.urdf";
        // urdf = "/home/pal/talos_data/example-robot-data/robots/talos_data/robots/talos_full_v2.urdf";
        std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
        std::string urdf = vm.count("fast") ? "talos/talos_fast.urdf" : "talos/talos.urdf";
        auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
        robot->set_position_enforced(vm["enforce_position"].as<bool>());
        if (vm["actuators"].as<std::string>() == "spd")
            robot->set_actuator_types("torque");
        else
            robot->set_actuator_types(vm["actuators"].as<std::string>());

        //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
        auto simu = std::make_shared<robot_dart::RobotDARTSimu>(dt);
        simu->set_collision_detector(vm["collision"].as<std::string>());

#ifdef GRAPHIC
        robot_dart::gui::magnum::GraphicsConfiguration configuration;
        if (vm.count("big_window")) {
            configuration.width = 1280;
            configuration.height = 960;
        }
        else {
            configuration.width = 1920;
            configuration.height = 1080;
        }

        auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
        simu->set_graphics(graphics);
        graphics->look_at({-2.5, 0.5, 3.2}, {0., 0., 1.4});
        if (vm.count("mp4"))
            graphics->record_video(vm["mp4"].as<std::string>());
#endif
        simu->add_robot(robot);
        auto floor = simu->add_checkerboard_floor();

        ///// CONTROLLER
        auto controller_path = vm["controller"].as<std::string>();
        auto controller_config = IWBC_CHECK(YAML::LoadFile(controller_path));
        // do some modifications according to command-line options
        controller_config["CONTROLLER"]["base_path"] = "../etc/talos"; // we assume that we run in ./build
        controller_config["CONTROLLER"]["urdf"] = robot->model_filename();
        controller_config["CONTROLLER"]["mimic_dof_names"] = robot->mimic_dof_names();
        controller_config["CONTROLLER"]["verbose"] = verbose;
        int control_freq = vm["control_freq"].as<int>();
        controller_config["CONTROLLER"]["dt"] = 1.0 / control_freq;
        auto controller_name = IWBC_CHECK(controller_config["CONTROLLER"]["name"].as<std::string>());
        auto closed_loop = IWBC_CHECK(controller_config["CONTROLLER"]["closed_loop"].as<bool>());
        if (vm.count("closed_loop")) {
            closed_loop = true;
            controller_config["CONTROLLER"]["closed_loop"] = true;
        }

        if (vm["actuators"].as<std::string>() == "torque" && !closed_loop)
            std::cout << "WARNING (iwbc): you should activate the closed loop if you are using torque control! (--closed_loop or yaml)" << std::endl;

        auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, controller_config);
        auto controller_pos = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
        IWBC_ASSERT(controller_pos, "we expect a PosTracker here");

        ///// BEHAVIOR
        auto behavior_path = vm["behavior"].as<std::string>();
        auto behavior_config = IWBC_CHECK(YAML::LoadFile(behavior_path));
        auto behavior_name = IWBC_CHECK(behavior_config["BEHAVIOR"]["name"].as<std::string>());
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller, behavior_config);
        IWBC_ASSERT(behavior, "invalid behavior");

        auto behavior_ = std::dynamic_pointer_cast<inria_wbc::behaviors::humanoid::FollowTrackers>(behavior);

        //uncomment to see spheres and trajectories of the robot


        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //initialize the vive tracking system////////////////////////////////////////////////////////////////////////////////////////
        inria::ViveTracking vive;
        vive.init("127.0.0.1","127.0.0.1");
        vive.update();

        // //for debugging
        // double t[512];
        // for (int i = 0;i < 512; i++)
        //     t[i] = i*2*M_PI/512;
        
        // double sin_t[512];
        // for (int i = 0;i < 512;i++)
        //     sin_t[i] = 0.2*std::sin(t[i]);

        // double cos_t[512];
        // for (int i = 0;i < 512;i++)
        //     cos_t[i] = 0.2*std::cos(t[i]);
        // ///////////////////////////////////

        //get both hands controllers
            //LHR-FC2F90A4 is for right tracker
            //LHR-21C1BC92 is for left tracker

        //vive initialization
        initialize_vive(vive);
        
        //int counter = 25;

        //then we can go forward
        //positions
        Eigen::Vector3d pos_vive_r = vive.get().at("LHR-FC2F90A4").posHand;
        Eigen::Vector3d pos_vive_l = vive.get().at("LHR-21C1BC92").posHand;

        // Eigen::Vector3d pos_vive_l = Eigen::Vector3d(1.70+sin_t[counter],1.85,0.42);
        // Eigen::Vector3d pos_vive_r = Eigen::Vector3d(1.20+sin_t[counter],1.63,0.77);

        //vive positions are not in the same referential as the robot, so we have to rotate them by a certain angle (different for each hand)

        //rotations
        Eigen::Matrix3d rot_vive_r = vive.get().at("LHR-FC2F90A4").matHand;
        Eigen::Matrix3d rot_vive_l = vive.get().at("LHR-21C1BC92").matHand;

        // Eigen::Matrix3d rot_vive_l = Eigen::AngleAxisd(1,Eigen::Vector3d::UnitY()).toRotationMatrix()
        //                             *Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d::UnitZ()).toRotationMatrix();

        //put the positions and the rotations into the correct referentials //////////
        //rot by pi to follow the good wrist rotations, to avoid flipping by pi the real wrist
        Eigen::Matrix3d rot_pi_rh = Eigen::AngleAxisd(M_PI,behavior_->get_init_rot_right()*Eigen::Vector3d::UnitZ()).toRotationMatrix();

        //get transformations for hands rotations
        Eigen::Matrix3d trans_r = get_trans(behavior_,vive,"LHR-FC2F90A4");
        Eigen::Matrix3d trans_l = get_trans(behavior_,vive,"LHR-21C1BC92");

        //get euler angles of vive rot because vie referentials are rotated by some theta in Z axis
        Eigen::Matrix3d rot_ref_r = get_rot_ref(vive,"LHR-FC2F90A4");
        Eigen::Matrix3d rot_ref_l = get_rot_ref(vive,"LHR-21C1BC92");

        //save init offsets
        Eigen::Vector3d init_offset_r = get_init_offset(behavior_->get_init_right(),vive,"LHR-FC2F90A4",K);
        Eigen::Vector3d init_offset_l = get_init_offset(behavior_->get_init_left(),vive,"LHR-21C1BC92",K);

        //recenter rh and lh positions
        Eigen::Vector3d pos_rh = vive_pos_processing(vive,"LHR-FC2F90A4",init_offset_r,rot_ref_r,K);
        Eigen::Vector3d pos_lh = vive_pos_processing(vive,"LHR-21C1BC92",init_offset_l,rot_ref_l,K);

        //get robot's hands rotations
        Eigen::Matrix3d rot_rh = vive_rot_processing(vive,"LHR-FC2F90A4",trans_r);
        Eigen::Matrix3d rot_lh = vive_rot_processing(vive,"LHR-21C1BC92",trans_l);

        //create spheres to represent the referentials
        //colors
        Eigen::Vector4d color_r(1,0,0,0.5);
        Eigen::Vector4d color_l(0,1,0,0.5);
        Eigen::Vector4d color_ref(0,0,1,0.5);

        //create spheres lists for both hands referentials
        std::vector<std::shared_ptr<robot_dart::Robot>> s_r_list;
        std::vector<std::shared_ptr<robot_dart::Robot>> s_l_list;

        //non calibrated referentials
        std::vector<std::shared_ptr<robot_dart::Robot>> s_r_nc_list;
        std::vector<std::shared_ptr<robot_dart::Robot>> s_l_nc_list;

        //hands
        std::vector<std::shared_ptr<robot_dart::Robot>> s_rh_list;
        std::vector<std::shared_ptr<robot_dart::Robot>> s_lh_list;

        //reference
        std::vector<std::shared_ptr<robot_dart::Robot>> s_ref_list;

        //draw them all
        draw_ref(pos_rh,rot_rh,color_r,s_r_list);
        draw_ref(pos_lh,rot_lh,color_l,s_l_list);
        draw_ref(pos_vive_r+Eigen::Vector3d(0,0,3),rot_vive_r,color_r,s_r_nc_list);
        draw_ref(pos_vive_l+Eigen::Vector3d(0,0,3),rot_vive_l,color_l,s_l_nc_list);
        draw_ref(behavior_->get_init_right(),behavior_->get_init_rot_right(),color_r,s_rh_list);
        draw_ref(behavior_->get_init_left(),behavior_->get_init_rot_left(),color_l,s_lh_list);
        draw_ref(Eigen::Vector3d(0,0,2.3),Eigen::Matrix3d::Identity(),color_ref,s_ref_list);

        
        //add them to the simulator
        // for (int i = 0;i < s_r_list.size();i++){
        //     // simu->add_visual_robot(s_r_list[i]);
        //     // simu->add_visual_robot(s_l_list[i]);
        //     // simu->add_visual_robot(s_r_nc_list[i]);
        //     // simu->add_visual_robot(s_l_nc_list[i]);
        //     // simu->add_visual_robot(s_rh_list[i]);
        //     // simu->add_visual_robot(s_lh_list[i]);
        //     // simu->add_visual_robot(s_ref_list[i]);
        // }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //generate exercises sequence /////////////////////////////////////////////////////////////////////////////
        std::vector<std::vector<Eigen::Vector3d>> exercises = behavior_->get_exercises();
        int index = 0;
        double save_time = (double) simu->scheduler().current_time();
        double max_duration = behavior_->get_max_duration();
        int number_of_penalties = 0;
        double dist_error = 0;
        std::vector<std::shared_ptr<robot_dart::Robot>> s_obj_list;
        draw_obj(s_obj_list,exercises,index);

        double epsilon = 1e-1; //accepts an error of 10cm maximum

        for (auto& elt : s_obj_list){
            simu->add_visual_robot(elt);
        }

        //draw distance between hand pose and goal
        std::vector<std::vector<std::shared_ptr<robot_dart::Robot>>> spheres_to_exercises;
        spheres_to_exercises.push_back(draw_dist_between_two_pos(exercises[0][index],robot->body_pose_vec("gripper_right_inner_double_link").tail<3>(),10));
        spheres_to_exercises.push_back(draw_dist_between_two_pos(exercises[1][index],robot->body_pose_vec("gripper_left_inner_double_link").tail<3>(),10));

        for (auto& sub_vec : spheres_to_exercises){
            for (auto& elt : sub_vec){
                simu->add_visual_robot(elt);
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        auto all_dofs = controller->all_dofs();
        auto floating_base = all_dofs;
        floating_base.resize(6);

        auto controllable_dofs = controller->controllable_dofs();
        robot->set_positions(controller->q0(), all_dofs);

        uint ncontrollable = controllable_dofs.size();

        if (vm.count("log")) {
            std::ofstream ofs("all_dofs.dat");
            for (auto& c : all_dofs)
                ofs << c << std::endl;
        }

        // add sensors to the robot
        auto ft_sensor_left = simu->add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint", control_freq, "parent_to_child");
        auto ft_sensor_right = simu->add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint", control_freq, "parent_to_child");
        robot_dart::sensor::IMUConfig imu_config;
        imu_config.body = robot->body_node("imu_link"); // choose which body the sensor is attached to
        imu_config.frequency = control_freq; // update rate of the sensor
        auto imu = simu->add_sensor<robot_dart::sensor::IMU>(imu_config);

        //////////////////// START SIMULATION //////////////////////////////////////
        simu->set_control_freq(control_freq); // default = 1000 Hz

        std::shared_ptr<robot_dart::Robot> ghost;
        if (vm.count("ghost") || vm.count("collisions")) {
            ghost = robot->clone_ghost();
            ghost->skeleton()->setPosition(4, -1.57);
            ghost->skeleton()->setPosition(5, 1.1);
            simu->add_robot(ghost);
        }

        // self-collision shapes
        std::vector<std::shared_ptr<robot_dart::Robot>> self_collision_spheres;
        if (vm.count("collisions")) {
            auto task_self_collision = controller_pos->task<tsid::tasks::TaskSelfCollision>(vm["collisions"].as<std::string>());
            for (size_t i = 0; i < task_self_collision->avoided_frames_positions().size(); ++i) {
                auto pos = task_self_collision->avoided_frames_positions()[i];
                auto tf = Eigen::Isometry3d(Eigen::Translation3d(pos[0], pos[1], pos[2]));
                double r0 = task_self_collision->avoided_frames_r0s()[i];
                auto sphere = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(r0 * 2, r0 * 2, r0 * 2), tf, "fixed", 1, Eigen::Vector4d(0, 1, 0, 0.5), "self-collision-" + std::to_string(i));
                sphere->set_color_mode("aspect");
                self_collision_spheres.push_back(sphere);
                simu->add_visual_robot(self_collision_spheres.back());
            }
        }
        std::vector<std::shared_ptr<robot_dart::sensor::Torque>> torque_sensors;

        auto talos_tracker_controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracker>(controller);
        for (const auto& joint : talos_tracker_controller->torque_sensor_joints()) {
            torque_sensors.push_back(simu->add_sensor<robot_dart::sensor::Torque>(robot, joint, control_freq));
            std::cerr << "Add joint torque sensor:  " << joint << std::endl;
        }

        // reading from sensors
        Eigen::VectorXd tq_sensors = Eigen::VectorXd::Zero(torque_sensors.size());

        // create the collision detectors (useful only if --check_self_collisions)
        inria_wbc::robot_dart::SelfCollisionDetector collision_detector(robot);
        std::map<std::string, std::string> filter_body_names_pairs;
        filter_body_names_pairs["leg_right_6_link"] = "BodyNode";
        filter_body_names_pairs["leg_left_6_link"] = "BodyNode";
        inria_wbc::robot_dart::ExternalCollisionDetector floor_collision_detector(robot, floor, filter_body_names_pairs);

        using namespace std::chrono;
        // to save trajectories
        std::shared_ptr<inria_wbc::trajs::Saver> traj_saver;
        if (!vm["save_traj"].empty()) {
            auto args = vm["save_traj"].as<std::vector<std::string>>();
            auto name = args[0];
            auto refs = std::vector<std::string>(args.begin() + 1, args.end());
            traj_saver = std::make_shared<inria_wbc::trajs::Saver>(controller_pos, args[0], refs);
        }
        // the main loop
        Eigen::VectorXd cmd;
        inria_wbc::controllers::SensorData sensor_data;
        inria_wbc::utils::Timer timer;
        auto active_dofs_controllable = controllable_dofs; // undamaged case
        auto active_dofs = controller->all_dofs(false); // false here: no filter at all
        inria_wbc::robot_dart::RobotDamages robot_damages(robot, simu, active_dofs_controllable, active_dofs);

        Eigen::VectorXd activated_joints = Eigen::VectorXd::Zero(active_dofs.size());

        bool init_model_sphere_collisions = false;
        std::vector<std::shared_ptr<robot_dart::Robot>> spheres;
        bool is_colliding = false;

        std::ofstream logfile_r("/home/pal/inria_wbc/vive_right_data__opti_scaled_sample1.csv");
        std::ofstream logfile_l("/home/pal/inria_wbc/vive_left_data__opti_scaled_sample1.csv");
        std::ofstream logfile_time("/home/pal/inria_wbc/time_record_file_opti_scaled.csv",std::ios::app);
        if (logfile_r){
            std::cout << "right open\n";
        }
        else{
            std::cout << "right not open\n";
            return 0;
        }

        if (logfile_l){
            std::cout << "left open\n";
        }
        else{
            std::cout << "left not open\n";
            return 0;
        }

        if (logfile_time){
            std::cout << "time open\n";
        }
        else{
            std::cout << "time not open\n";
            return 0;
        }

        //BEGINNING OF THE LOOP /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        while (simu->scheduler().next_time() < 1000 /*vm["duration"].as<int>()*/ && !simu->graphics()->done() && index < exercises[0].size()) {

            if (vm["damage"].as<bool>()) {
                try {

                    if (simu->scheduler().current_time() == 0.0) {
                        collision_detector.remove_frames();
                        robot_damages.cut(vm["cut"].as<std::string>());
                        collision_detector.add_frames();
                        active_dofs_controllable = robot_damages.active_dofs_controllable();
                        active_dofs = robot_damages.active_dofs();
                    }
                }
                catch (std::exception& e) {
                    std::cout << red << bold << "Error (exception): " << rst << e.what() << std::endl;
                }
            }

            if (vm.count("check_self_collisions")) {
                IWBC_ASSERT(!vm.count("fast"), "=> check_self_collisions is not compatible with --fast!");
                auto collision_list = collision_detector.collide();
                if (!collision_list.empty())
                    std::cout << " ------ SELF Collisions ------ " << std::endl;
                for (auto& s : collision_list)
                    std::cout << s << std::endl;
            }

            if (vm.count("check_fall")) {
                auto head_z_diff = std::abs(controller->model_frame_pos("head_1_link").translation()(2) - robot->body_pose("head_1_link").translation()(2));
                std::vector<std::string> floor_collision_list;
                if (head_z_diff > 0.75)
                    floor_collision_list.push_back("head_1_link");
                if (!vm.count("fast")) {
                    auto list2 = floor_collision_detector.collide();
                    floor_collision_list.insert(floor_collision_list.end(), list2.begin(), list2.end());
                }
                if (!floor_collision_list.empty())
                    std::cout << " ------ FLOOR Collisions ------ " << std::endl;
                for (auto& s : floor_collision_list)
                    std::cout << s << std::endl;
            }

            //check if has fallen
            if (robot->body_pose_vec("head_1_link").tail<3>().coeff(2) < 0.5)
            {
                return 100000;
            }
            

            // get actual torque from sensors
            for (size_t i = 0; i < torque_sensors.size(); ++i)
                if (torque_sensors[i]->active())
                    tq_sensors(i) = torque_sensors[i]->torques()(0, 0);
                else
                    tq_sensors(i) = 0;

            if (vm["height"].as<bool>() && ft_sensor_left->active() && ft_sensor_right->active())
                std::cout << controller->t() << "  floating base height: " << controller->q(false)[2] << " - total feet force: " << ft_sensor_right->force().norm() + ft_sensor_left->force().norm() << std::endl;

            // step the command
            if (simu->schedule(simu->control_freq())) {

                // update the sensors
                // left foot
                if (ft_sensor_left->active()) {
                    sensor_data["lf_torque"] = ft_sensor_left->torque();
                    sensor_data["lf_force"] = ft_sensor_left->force();
                }
                else {
                    sensor_data["lf_torque"] = Eigen::VectorXd::Constant(3, 1e-8);
                    sensor_data["lf_force"] = Eigen::VectorXd::Constant(3, 1e-8);
                }
                // right foot
                if (ft_sensor_right->active()) {
                    sensor_data["rf_torque"] = ft_sensor_right->torque();
                    sensor_data["rf_force"] = ft_sensor_right->force();
                }
                else {
                    sensor_data["rf_torque"] = Eigen::VectorXd::Constant(3, 1e-8);
                    sensor_data["rf_force"] = Eigen::VectorXd::Constant(3, 1e-8);
                }
                // accelerometer

                sensor_data["imu_pos"] = imu->angular_position_vec();
                sensor_data["imu_vel"] = imu->angular_velocity();
                sensor_data["imu_acc"] = imu->linear_acceleration();
                sensor_data["velocity"] = robot->com_velocity().tail<3>();
                // joint positions / velocities (excluding floating base)
                // 0 for joints that are not in active_dofs_controllable
                Eigen::VectorXd positions = Eigen::VectorXd::Zero(controller->controllable_dofs(false).size());
                Eigen::VectorXd velocities = Eigen::VectorXd::Zero(controller->controllable_dofs(false).size());
                for (size_t i = 0; i < controller->controllable_dofs(false).size(); ++i) {
                    auto name = controller->controllable_dofs(false)[i];
                    if (std::count(active_dofs_controllable.begin(), active_dofs_controllable.end(), name) > 0) {
                        positions(i) = robot->positions({name})[0];
                        velocities(i) = robot->velocities({name})[0];
                    }
                }
                sensor_data["positions"] = positions;
                sensor_data["joints_torque"] = tq_sensors;
                sensor_data["joint_velocities"] = velocities;
                // floating base (perfect: no noise in the estimate)
                sensor_data["floating_base_position"] = inria_wbc::robot_dart::floating_base_pos(robot->positions());
                sensor_data["floating_base_velocity"] = inria_wbc::robot_dart::floating_base_vel(robot->velocities());

                timer.begin("solver");
                behavior->update(sensor_data);
                auto q = controller->q(false);
                timer.end("solver");

                Eigen::VectorXd q_no_mimic = controller->filter_cmd(q).tail(ncontrollable); //no fb
                timer.begin("cmd");
                Eigen::VectorXd q_damaged = inria_wbc::robot_dart::filter_cmd(q_no_mimic, controllable_dofs, active_dofs_controllable);

                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = inria_wbc::robot_dart::compute_velocities(robot, q_damaged, 1. / control_freq, active_dofs_controllable);
                else if (vm["actuators"].as<std::string>() == "spd") {
                    cmd = inria_wbc::robot_dart::compute_spd(robot, q_damaged, 1. / sim_freq, active_dofs_controllable, false);
                }
                else { // torque
                    Eigen::VectorXd cmd_no_mimic = controller->filter_cmd(controller->tau(false)).tail(ncontrollable);
                    cmd = inria_wbc::robot_dart::filter_cmd(cmd_no_mimic, controllable_dofs, active_dofs_controllable);
                }
                timer.end("cmd");

                Eigen::VectorXd translate_ghost = Eigen::VectorXd::Zero(6);
                if (ghost) {
                    translate_ghost(0) -= 1;
                    ghost->set_positions(controller->filter_cmd(controller->q_solver(false)).tail(ncontrollable), controllable_dofs);
                    ghost->set_positions(controller->q_solver(false).head(6) + translate_ghost, floating_base);
                }

                is_colliding = controller->is_model_colliding();
                if (vm["model_collisions"].as<bool>()) {
                    auto spherical_members = controller->collision_check().spherical_members();
                    auto sphere_color = dart::Color::Green(0.5);

                    if (init_model_sphere_collisions == false) {
                        spheres = inria_wbc::robot_dart::create_spherical_members(spherical_members, *simu, sphere_color);
                        init_model_sphere_collisions = true;
                    }
                    else {
                        inria_wbc::robot_dart::update_spherical_members(spherical_members, spheres, sphere_color, is_colliding, controller->collision_check().collision_index(), translate_ghost.head(3));
                    }
                }
            }

            //add a penalty if robot is supposedly colliding
            if (is_colliding)
                number_of_penalties++;

            if (simu->schedule(simu->graphics_freq()) && vm.count("collisions")) {
                auto controller_pos = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
                auto task_self_collision = controller_pos->task<tsid::tasks::TaskSelfCollision>(vm["collisions"].as<std::string>());
                for (size_t i = 0; i < task_self_collision->avoided_frames_positions().size(); ++i) {
                    auto cp = self_collision_spheres[i]->base_pose();
                    cp.translation() = task_self_collision->avoided_frames_positions()[i];
                    cp.translation()[0] -= 1; // move to the ghost
                    self_collision_spheres[i]->set_base_pose(cp);
                    auto bd = self_collision_spheres[i]->skeleton()->getBodyNodes()[0];
                    auto visual = bd->getShapeNodesWith<dart::dynamics::VisualAspect>()[0];
                    visual->getShape()->setDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
                    bool c = task_self_collision->collision(i);
                    if (c) {
                        visual->getVisualAspect()->setRGBA(dart::Color::Red(1.0));
                    }
                    else {
                        visual->getVisualAspect()->setRGBA(dart::Color::Green(1.0));
                    }
                }
            }

            // push the robot
            bool push = false;
            if (vm.count("push")) {
                auto pv = vm["push"].as<std::vector<float>>();
                auto pforce = vm["norm_force"].as<float>();
                for (auto& p : pv) {
                    if (simu->scheduler().current_time() > p && simu->scheduler().current_time() < p + 0.5) {
                        robot->set_external_force("base_link", Eigen::Vector3d(0, pforce, 0));
                        // robot->set_external_force("base_link", Eigen::Vector3d(pforce, 0, 0));
                        push = true;
                    }
                    if (simu->scheduler().current_time() > p + 0.25)
                        robot->clear_external_forces();
                }
            }

            //update tracking motion of each hand only if corresponding trigger is pulled
            bool test = vive.update();
            
            //std::cout << "droite: " << vive.get().at("LHR-FC2F90A4").isButtonTrigger << " gauche: " << vive.get().at("LHR-21C1BC92").isButtonTrigger << std::endl;
            //if new calculated right position is valid
            if (vive.get().at("LHR-FC2F90A4").isValid && test){
                //counter = (counter+1)%512;
                //then update the spheres positions in the simulator
                pos_vive_r = vive.get().at("LHR-FC2F90A4").posHand;
                //pos_vive_r = Eigen::Vector3d(1.20+sin_t[counter],1.63,0.77);

                //same for rotations
                rot_vive_r = vive.get().at("LHR-FC2F90A4").matHand;
                // rot_vive_r = Eigen::AngleAxisd(1,Eigen::Vector3d::UnitY()).toRotationMatrix()
                //             *Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d::UnitZ()).toRotationMatrix();

                //pos_vive_r = rot_vive_r*pos_vive_r;
                
                //robot's right hand rotation matrix
                rot_rh = vive_rot_processing(vive,"LHR-FC2F90A4",trans_r);

                //put position in the good rotation and then substract the offset
                pos_rh = vive_pos_processing(vive,"LHR-FC2F90A4",init_offset_r,rot_ref_r,K);
                
            }

            //if new calculated left position is valid
            if (vive.get().at("LHR-21C1BC92").isValid && test){
        
                // //then update the spheres positions in the simulator
                pos_vive_l = vive.get().at("LHR-21C1BC92").posHand;
                //pos_vive_l = Eigen::Vector3d(1.70+sin_t[counter],1.85,0.42);

                // //same for rotations
                rot_vive_l = vive.get().at("LHR-21C1BC92").matHand;
                // rot_vive_l = Eigen::AngleAxisd(1,Eigen::Vector3d::UnitY()).toRotationMatrix()
                //             *Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d::UnitZ()).toRotationMatrix();

                //pos_vive_l = rot_vive_l*pos_vive_l;

                //left hand's rotation matrix
                rot_lh = vive_rot_processing(vive,"LHR-21C1BC92",trans_l);

                //put vive position in the good rotation and then substract the offset
                pos_lh = vive_pos_processing(vive,"LHR-21C1BC92",init_offset_l,rot_ref_l,K);

            }

            // log data in files
            logfile_l << "x: " << pos_lh.coeff(0)
                      << " y: " << pos_lh.coeff(1)
                      << " z: " << pos_lh.coeff(2)
                      << " hx: " << robot->body_pose_vec("gripper_left_inner_double_link").tail<3>().coeff(0)
                      << " hy: " << robot->body_pose_vec("gripper_left_inner_double_link").tail<3>().coeff(1)
                      << " hz: " << robot->body_pose_vec("gripper_left_inner_double_link").tail<3>().coeff(2)
                      << " valide: " << vive.get().at("LHR-21C1BC92").isValid << std::endl;

            logfile_r << "x: " << pos_rh.coeff(0)
                      << " y: " << pos_rh.coeff(1)
                      << " z: " << pos_rh.coeff(2)
                      << " hx: " << robot->body_pose_vec("gripper_right_inner_double_link").tail<3>().coeff(0)
                      << " hy: " << robot->body_pose_vec("gripper_right_inner_double_link").tail<3>().coeff(1)
                      << " hz: " << robot->body_pose_vec("gripper_right_inner_double_link").tail<3>().coeff(2)
                      << " valid: " << vive.get().at("LHR-FC2F90A4").isValid << std::endl;

            //update vive spheres positions
            // update_ref(pos_rh,rot_rh,s_r_list);
            // update_ref(pos_lh,rot_lh,s_l_list);
            // update_ref(pos_vive_r+Eigen::Vector3d(0,0,3),rot_vive_r,s_r_nc_list);
            // update_ref(pos_vive_l+Eigen::Vector3d(0,0,3),rot_vive_l,s_l_nc_list);

            if (test)
                behavior_->update_trajectories(pos_rh,pos_lh,rot_rh,rot_lh);

            //if objective achieved, go to the next one (it loops)
            if ((is_obj_achieved(exercises[0][index],robot->body_pose_vec("gripper_right_inner_double_link").tail<3>(),epsilon) 
                && is_obj_achieved(exercises[1][index],robot->body_pose_vec("gripper_left_inner_double_link").tail<3>(),epsilon))
                || (double)simu->scheduler().current_time() - save_time > max_duration)
            {
                dist_error += calculate_dist_error_by_dir(exercises[0][index],robot->body_pose_vec("gripper_right_inner_double_link").tail<3>());
                dist_error += calculate_dist_error_by_dir(exercises[1][index],robot->body_pose_vec("gripper_left_inner_double_link").tail<3>());

                index++;
                if ((double)simu->scheduler().current_time() - save_time > max_duration){
                    number_of_penalties++;
                }
                save_time = (double) simu->scheduler().current_time();
                update_obj(s_obj_list,exercises,index);
            }

            //update distance between hand pose and goal
            update_dist_between_two_pos(exercises[0][index],robot->body_pose_vec("gripper_right_inner_double_link").tail<3>(),spheres_to_exercises[0]);
            update_dist_between_two_pos(exercises[1][index],robot->body_pose_vec("gripper_left_inner_double_link").tail<3>(),spheres_to_exercises[1]);

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // step the simulation
            {
                timer.begin("sim");
                robot->set_commands(cmd, active_dofs_controllable);
                simu->step_world();
                timer.end("sim");

                // auto col = simu->world()->getConstraintSolver()->getLastCollisionResult();
                // size_t nc = col.getNumContacts();
                // size_t contact_count = 0;
                // for (size_t i = 0; i < nc; i++) {
                //     auto& ct = col.getContact(i);
                //     auto f1 = ct.collisionObject1->getShapeFrame();
                //     auto f2 = ct.collisionObject2->getShapeFrame();
                //     std::string name1, name2;
                //     if (f1->isShapeNode())
                //         name1 = f1->asShapeNode()->getBodyNodePtr()->getName();
                //     if (f1->isShapeNode())
                //         name2 = f1->asShapeNode()->getBodyNodePtr()->getName();
                //     std::cout << "contact:" << name1<< " -- " << name2 << std::endl;
                // }
            }

            if (traj_saver)
                traj_saver->update();
            // log if needed
            for (auto& x : log_files) {
                if (x.first == "timing")
                    timer.report(*x.second, simu->scheduler().current_time());
                else if (x.first == "cmd")
                    (*x.second) << cmd.transpose() << std::endl;
                else if (x.first == "tau")
                    (*x.second) << controller->tau().transpose() << std::endl;
                else if (x.first == "com") // the real com
                    (*x.second) << robot->com().transpose() << std::endl;
                else if (x.first == "controller_com") // the com according to controller
                    (*x.second) << controller->com().transpose() << std::endl;
                else if (x.first == "objective_value")
                    (*x.second) << controller_pos->objective_value() << std::endl;
                else if (x.first.find("cost_") != std::string::npos) // e.g. cost_com
                    (*x.second) << controller->cost(x.first.substr(strlen("cost_"))) << std::endl;
                else if (x.first == "ft")
                    (*x.second) << ft_sensor_left->torque().transpose() << " " << ft_sensor_left->force().transpose() << " "
                                << ft_sensor_right->torque().transpose() << " " << ft_sensor_right->force().transpose() << std::endl;
                else if (x.first == "force") // the cop according to controller
                    (*x.second) << ft_sensor_left->force().transpose() << " "
                                << controller->lf_force_filtered().transpose() << " "
                                << ft_sensor_right->force().transpose() << " "
                                << controller->rf_force_filtered().transpose() << std::endl;
                else if (x.first == "controller_momentum") // the momentum according to pinocchio
                    (*x.second) << controller->momentum().transpose() << std::endl;
                else if (x.first == "imu") // the momentum according to pinocchio
                    (*x.second) << imu->angular_position_vec().transpose() << " "
                                << imu->angular_velocity().transpose() << " "
                                << imu->linear_acceleration().transpose() << std::endl;
                else if (x.first == "controller_imu") // the momentum according to pinocchio
                    (*x.second) << controller->robot()->framePosition(controller->tsid()->data(), controller->robot()->model().getFrameId("imu_link")).translation().transpose() << " "
                                << controller->robot()->frameVelocityWorldOriented(controller->tsid()->data(), controller->robot()->model().getFrameId("imu_link")).angular().transpose() << " "
                                << controller->robot()->frameAccelerationWorldOriented(controller->tsid()->data(), controller->robot()->model().getFrameId("imu_link")).linear().transpose() << std::endl;
                else if (x.first == "com_vel")
                    (*x.second) << robot->skeleton()->getCOMSpatialVelocity().head(3).transpose() << std::endl;
                else if (x.first == "momentum") {
                    auto bodies = robot->body_names();
                    Eigen::Vector3d angular_momentum = Eigen::Vector3d::Zero();
                    for (auto& b : bodies)
                        angular_momentum += robot->body_node(b)->getAngularMomentum(robot->com());
                    (*x.second) << -angular_momentum.transpose() << std::endl;
                }
                else if (x.first == "cop") { // the cop according to controller
                    if (controller->cop())
                        (*x.second) << controller->cop().value().transpose() << std::endl;
                    else
                        (*x.second) << Eigen::Vector2d::Constant(1000).transpose() << std::endl;
                }
                else if (x.first == "lcop") { // the cop according to controller
                    if (controller->lcop())
                        (*x.second) << controller->lcop().value().transpose() << std::endl;
                    else
                        (*x.second) << Eigen::Vector2d::Constant(1000).transpose() << std::endl;
                }
                else if (x.first == "rcop") { // the cop according to controller
                    if (controller->rcop())
                        (*x.second) << controller->rcop().value().transpose() << std::endl;
                    else
                        (*x.second) << Eigen::Vector2d::Constant(1000).transpose() << std::endl;
                }
                else if (x.first.find("task_") != std::string::npos) // e.g. task_lh
                {
                    auto ref = controller_pos->se3_task(x.first.substr(strlen("task_")))->getReference();
                    (*x.second) << ref.getValue().transpose() << " "
                                << ref.getDerivative().transpose() << " "
                                << ref.getSecondDerivative().transpose() << std::endl;
                }
                else if (robot->body_node(x.first) != nullptr) {
                    pinocchio::SE3 frame;
                    frame.rotation() = robot->body_pose(x.first).rotation();
                    frame.translation() = robot->body_pose(x.first).translation();

                    Eigen::VectorXd vec(12);
                    tsid::math::SE3ToVector(frame, vec);
                    (*x.second) << vec.transpose() << std::endl;
                }
            }
            if (vm.count("srdf")) {
                auto conf = vm["srdf"].as<float>();
                if (controller->t() >= conf && controller->t() < conf + controller->dt())
                    controller->save_configuration("configuration.srdf");
            }
            if (timer.iteration() == 100) {
                std::ostringstream oss;
#ifdef GRAPHIC // to avoid the warning
                oss.precision(3);
                timer.report(oss, simu->scheduler().current_time(), -1, '\n');
                if (is_colliding)
                    oss << "Model is colliding" << std::endl;
                if (!vm.count("mp4"))
                    simu->set_text_panel(oss.str());
#endif
            }
            timer.report(simu->scheduler().current_time(), 100);
        }

        int finished;
        if (index == exercises[0].size())
            finished = 1;
        else
            finished = 0;

        double time_spent = (double)simu->scheduler().current_time();

        logfile_time << "time: " << time_spent << " completed: " << finished << std::endl;
        logfile_time.close();
        logfile_r.close();
        logfile_l.close();

        if (index == exercises[0].size())
            std::cout << "you completed all the exercises, congrats!!!!\n" << std::endl;

        return note(time_spent,number_of_penalties,dist_error);
    }
    catch (YAML::RepresentationException& e) {
        std::cout << red << bold << "YAML Parse error (missing key in YAML file?): " << rst << e.what() << std::endl;
    }
    catch (YAML::ParserException& e) {
        std::cout << red << bold << "YAML Parse error: " << rst << e.what() << std::endl;
    }
    catch (std::exception& e) {
        std::cout << red << bold << "Error (exception): " << rst << e.what() << std::endl;
    }
}

