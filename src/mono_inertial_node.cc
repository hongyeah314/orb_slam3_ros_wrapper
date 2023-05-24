/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
*
*/

#include "common.h"
#include "custom_msgs/Encoder.h"


using namespace std;

class ImuGrabber  //IMU 数据 grab类
{
public:
    ImuGrabber()= default;
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class EncoderGrabber
{
public:

    EncoderGrabber()= default;
//    void GrabEncoder(const custom_msgs::EncoderConstPtr &encoder_msg);
    void GrabEncoder(const geometry_msgs::TwistStampedPtr &encoder_msg);
//    queue<custom_msgs::EncoderConstPtr> encbuf;
    queue<geometry_msgs::TwistStampedPtr> encbuf;
    std::mutex mBufMutex;

};

class ImageGrabber  //图像数据 grab类
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb): mpSLAM(pSLAM), mpImuGb(pImuGb){}
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, EncoderGrabber *pEncGb): mpSLAM(pSLAM), mpImuGb(pImuGb), mpEncGb(pEncGb){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();  //Image 数据和IMU数据进行时间戳对齐

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    EncoderGrabber *mpEncGb;
};

//bool buseEncoder;
bool buseEncoder = true;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;
    const std::string& node_name = ros::this_node::getName();
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);

    // 利用一个bool变量来辨别是否融合轮速数据


    EncoderGrabber encgb;

    cerr<<"sub encoder"<<endl;
    ros::Subscriber sub_encoder = node_handler.subscribe("/twiststamped", 100, &EncoderGrabber::GrabEncoder,&encgb);


    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, &encgb);
    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img0 = node_handler.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &igb);
    setup_ros_publishers(node_handler, image_transport);
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    //cerr<<" add img "<<endl;
    mBufMutex.unlock();
}



cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
//        cerr<<"buseEncoder: "<<buseEncoder<<endl;
        if(!buseEncoder){
        if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
        {
            cv::Mat im;
            double tIm = 0;

            tIm = img0Buf.front()->header.stamp.toSec();   //最近一张图像的时间戳
            if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())  //如果当前IMU的时间比IMG数据晚
                continue;

            cerr<<"begin to get image"<<endl;
            this->mBufMutex.lock();
            im = GetImage(img0Buf.front()); //把最前面的image提出来

            ros::Time msg_time = img0Buf.front()->header.stamp;
            img0Buf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

                    vImuMeas.emplace_back(acc, gyr, t);//emplace_back的参数是作为构造函数参数的序列，它在容器的尾部直接构造一个新元素。
                            // 这种方法避免了拷贝构造函数的调用，因为它直接在容器中构造元素。这样可以提高效率并消除复制操作，使代码更加简洁。

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // Main algorithm runs here
            cerr<<"begin tracking"<<endl;
            Sophus::SE3f Tcw = mpSLAM->TrackMonocular(im, tIm, vImuMeas);
            Sophus::SE3f Twc = Tcw.inverse();
            
            publish_ros_camera_pose(Twc, msg_time);
            publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
            publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
        }
        }
        else{
//            cerr<<"imgBuffer size:  "<<img0Buf.size()<<endl;
//            cerr<<"imuBuffer size:  "<<mpImuGb->imuBuf.size()<<endl;
            //cerr<<"EncBuffer size:  "<<mpEncGb->encbuf.size()<<endl;
            if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty()&&!mpEncGb->encbuf.empty())
            {
                cv::Mat im;
                double tIm = 0;

                tIm = img0Buf.front()->header.stamp.toSec();   //最近一张图像的时间戳
                //cerr<<"图像的时间戳: "<< tIm <<endl;
                //cerr<<"IMU的时间戳: "<< mpImuGb->imuBuf.back()->header.stamp.toSec()<<endl;

                if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())  //如果当前IMU的时间比IMG数据晚
                    continue;
                //cerr<<"begin to get image"<<endl;
                this->mBufMutex.lock();
                im = GetImage(img0Buf.front()); //把最前面的image提出来
                ros::Time msg_time = img0Buf.front()->header.stamp;
                img0Buf.pop();
                //cerr<<"finishing getting image"<<endl;
                this->mBufMutex.unlock();
                //cerr<<"begin to process IMU"<<endl;
                vector<ORB_SLAM3::IMU::Point> vImuMeas;
                mpImuGb->mBufMutex.lock();
                mpEncGb->mBufMutex.lock();
                if (!mpImuGb->imuBuf.empty())
                {
                    // Load imu measurements from buffer
                    vImuMeas.clear();
                    //cerr<<"Imu Buffer: "<<mpImuGb->imuBuf.size()<<endl;
                    //cerr<<"进入循环 ： "<<(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)<<endl;
                    while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)      //找出离该IMU最近的一帧encoder
                    {
                        double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                        double encoder_v = 0;
                        //cerr<<"Encoder Buffer: "<<mpEncGb->encbuf.size()<<endl;
                        if(mpEncGb->encbuf.empty()) encoder_v=vImuMeas.back().encoder_v;
                        else {
                            while(!mpEncGb->encbuf.empty()&&mpEncGb->encbuf.front()->header.stamp.toSec()<=mpImuGb->imuBuf.front()->header.stamp.toSec()){
 //                               d_encoder = 0

                                encoder_v=mpEncGb->encbuf.front()->twist.linear.x;
                                mpEncGb->encbuf.pop();
                            }
                        }


                        cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);

                        cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

                        vImuMeas.emplace_back(acc, gyr, t,encoder_v);//emplace_back的参数是作为构造函数参数的序列，它在容器的尾部直接构造一个新元素。
                        //cerr<<"当前帧的速度为： "<<encoder_v<<endl;
                        // 这种方法避免了拷贝构造函数的调用，因为它直接在容器中构造元素。这样可以提高效率并消除复制操作，使代码更加简洁。

                        mpImuGb->imuBuf.pop();
                    }
                }
                mpImuGb->mBufMutex.unlock();
                mpEncGb->mBufMutex.unlock();

                cerr<<"begin tracking"<<endl;
                // Main algorithm runs here
                Sophus::SE3f Tcw = mpSLAM->TrackMonocular(im, tIm, vImuMeas);
                Sophus::SE3f Twc = Tcw.inverse();

                publish_ros_camera_pose(Twc, msg_time);
                publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
                publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
            }
        }
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
}

void EncoderGrabber::GrabEncoder(const geometry_msgs::TwistStampedPtr &encoder_msg)
{
    //cerr<<"Begin to receive Encoder"<<endl;
    mBufMutex.lock();

    encbuf.push(encoder_msg);

    mBufMutex.unlock();
}