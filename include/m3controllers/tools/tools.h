#ifndef TOOLS_H
#define TOOLS_H

////////// NOTE
// Functions with body must be defined as inline otherwise the factory will not compile

////////// STD
#include <iostream>
#include <fstream> 
#include <iterator>

////////// YAML-CPP
#include <yaml-cpp/yaml.h>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>

////////// M3RT
#include <m3rt/base/toolbox.h>

////////// Eigen3
#include <eigen3/Eigen/Core>

#define USE_ROS_RT_PUBLISHER //HACK

#ifdef USE_ROS_RT_PUBLISHER
////////// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <realtime_tools/realtime_publisher.h>
#endif

////////// Some defs
#define mm2m(a)	(mReal((a))/1000) //millimeters to meters
#define m2mm(a)	(mReal((a))*1000) //meters to millimeters
#define M3_CONTROLLERS_LOG "/home/gennaro/log_files/m3controllers/" //FIXME

namespace tools {
	/*
inline bool GenerateDocFromCfg(const char* cfg_filename, YAML::Node& doc)
{       std::string path;
	std::string file_name(cfg_filename);
	std::string cur_path = boost::filesystem::current_path().string();
	cur_path.append("/mekabot/m3ens/m3_controllers/"); //FIX
	path.append(cur_path);
	path.append(file_name);
	std::ifstream fin(path.c_str());
	if (fin.fail())
	{		
		m3rt::M3_ERR("could not read %s \n", path.c_str());
		return false;
	}
   	YAML::Parser parser(fin);
   	parser.GetNextDocument(doc);
	fin.close();
	
	return true;
}
*/
template<typename value_t>
struct JointLimits{
	value_t min_q;
	value_t max_q;
};



class MinJerk
{
	public:
		MinJerk():x(0),v(0),p(0),j(0){}
		inline double GetX() const {return x;}
		inline double GetXDot() const {return v;}
		inline double GetXDotDot() const {return p;}
		inline double GetXDotDotDot() const {return j;}
		
		inline void Compute(const double& t)
		{
		    tau=(t-t0)/D;
		    
		    if (tau>1.0)
		    {
			    x=xf;v=0.0;p=0.0;j=0.0;
			    return;
		    }
		    x=	  a0 + 
			    a1*tau +
			    a2*pow(tau,2)+
			    a3*pow(tau,3)+
			    a4*pow(tau,4)+
			    a5*pow(tau,5);
		    /*if (tmp_cnt++%100==0)
			    M3_INFO("T: %f Tau: %f ",t,tau);
		    if (tmp_cnt%100==0)	
			    M3_INFO("X %f\n",x);*/
		    
		    v=	  a1/D +
			    2*a2*tau/D+
			    3*a3*pow(tau,2)/D+
			    4*a4*pow(tau,3)/D+
			    5*a5*pow(tau,4)/D;
			    
		    p=	  2*a2/pow(D,2)+
			    6*a3*tau/pow(D,2)+
			    12*a4*pow(tau,2)/pow(D,2)+
			    20*a5*pow(tau,3)/pow(D,2);
		    
		    j=	  6*a3/pow(D,3)+
			    24*a4*tau/pow(D,3)+
			    60*a5*pow(tau,2)/pow(D,3);
		}
		
		inline void Create(double mxi,double mvi,double mpi,double mxf,double mt0,double mtf)
		{
		    t0=mt0;
		    tf=mtf;
		    xf=mxf;
		    D=mtf-mt0;
		    a0=mxi;
		    a1=D*mvi;
		    a2=pow(D,2)*mpi/2.0;
		    a3=-1.5*mpi*pow(D,2) - 6*mvi*D + 10*(mxf-mxi);
		    a4= 1.5*mpi*pow(D,2) + 8*mvi*D - 15*(mxf-mxi);
		    a5=-0.5*mpi*pow(D,2) - 3*mvi*D +  6*(mxf-mxi);
		    /*M3_INFO("Create t0 %f\n",t0);
		    M3_INFO("Create tf %f\n",tf);
		    M3_INFO("Create xf %f\n",mxf);
		    M3_INFO("Create xi %f\n",mxi);
		    M3_INFO("Create vi %f\n",mvi);
		    M3_INFO("Create pi %f\n",mpi);
		    M3_INFO("Create a1 %f\n",a1);
		    M3_INFO("Create a2 %f\n",a2);
		    M3_INFO("Create a3 %f\n",a3);
		    M3_INFO("Create a4 %f\n",a4);
		    M3_INFO("Create a5 %f\n",a5);*/
		}
		
	private:
		double t0,tf,xf,D,a0,a1,a2,a3,a4,a5;
		double x,v,p,j;
		double tau;
};  

template<typename value_t>
void ReadTxtFile(const char* filename,std::vector<std::vector<value_t> >& values ) {
    std::string line;
    values.clear();
    std::ifstream myfile (filename);
    std::istringstream iss;
    std::size_t i=0;
    std::size_t nb_vals=0;
    if (myfile.is_open())
    {
        while (getline(myfile,line)) {
            values.push_back(std::vector<value_t>());;
            std::vector<value_t>& v = values[i];
            iss.clear();
            iss.str(line);
            std::copy(std::istream_iterator<value_t>(iss),std::istream_iterator<value_t>(), std::back_inserter(v));
            nb_vals+=v.size();
            i++;
        }
	std::cout << "File ["<<filename<<"] read with success  ["<<nb_vals<<" values, "<<i<<" lines] "<<std::endl;
    }
    else{
	 std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}

template<typename value_t>
void WriteTxtFile(const char* filename, std::vector<std::vector<value_t> >& values) {
    std::ofstream myfile(filename);    
    std::size_t row = 0;
    std::size_t col = 0;
    std::size_t nb_rows = values.size();
    std::size_t nb_cols = values[0].size();
    if (myfile.is_open())
    {
        while(row < nb_rows) {
	    while(col < nb_cols){
		if (col == nb_cols-1)
			myfile << values[row][col] << "\n";
		else
			myfile << values[row][col] << " ";
		col++;  
	 }
	    col=0;
            row++;
        }
	std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows, "<<nb_cols<<" cols] "<<std::endl;
    }
    else{
	 std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}

template<typename value_t>
void WriteTxtFile(const char* filename, std::vector<value_t>& values) {
    std::ofstream myfile (filename);
    std::size_t row = 0;
    std::size_t nb_rows = values.size();
    if (myfile.is_open())
    {
        while(row < nb_rows) {
	    myfile << values[row] << "\n";
            row++;
        }
	std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows ] "<<std::endl;
    }
    else{
	 std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}



typedef std::vector<std::vector<mReal> > matrix_t;
typedef std::vector<mReal> vector_t;
class FileDumper
{
	public:
		FileDumper(const int rows,const int cols, const std::string file_name, const std::string file_path)
		{
			assert(rows > 0);
			assert(cols > 0);
			rows_ = rows;
			cols_ = cols;
			file_name_ = file_name;
			file_path_ = file_path;
			rows_dumped_ = 0;
			for(int i = 0; i<rows_; i++)
				matrix_.push_back(vector_t(cols_));
		}
		void Dump()
		{
			boost::filesystem::path dir(file_path_);
			if(!boost::filesystem::is_directory(dir))
				boost::filesystem::create_directories(dir);
			tools::WriteTxtFile((file_path_+"/"+file_name_+".txt").c_str(),matrix_);
		}
		void Save(const Eigen::Ref<const Eigen::VectorXd>& vector)
		{
			if(rows_dumped_ < rows_)
			{ 
				Eigen::Map<Eigen::VectorXd>(&matrix_[rows_dumped_][0],cols_) = vector;
				rows_dumped_++;
			}
		}
		std::string getFileName()
		{
			return file_name_;
		}
		
	private:
		matrix_t matrix_;
		int rows_, cols_, rows_dumped_;
		std::string file_name_, file_path_;
};

typedef std::map<std::string,std::pair<Eigen::VectorXd*,FileDumper*> > map_t;
typedef map_t::iterator map_it_t;
class FileDumpers
{
	public:
		FileDumpers(){};
		~FileDumpers()
		{
			for(map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
				delete iterator->second.second;
		}
		void AddDumper(const int rows, Eigen::VectorXd* vector_ptr, const std::string file_name, const std::string file_path)
		{
			// Create a fresh FileDumper
			FileDumper* file_dumper_ptr = new FileDumper(rows,vector_ptr->size(),file_name,file_path);
			
			// Put it into the map with his friend
			map_[file_name] = std::make_pair(vector_ptr,file_dumper_ptr);
		}
		//void Clear()
		//{
		//	map_.clear();
		//}
		void DumpAll()
		{
			for(map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
				iterator->second.second->Dump();
		}
		
		void SaveAll()
		{
			for(map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
				iterator->second.second->Save(*iterator->second.first);
		}
		
	private:
		map_t map_;
};

#ifdef USE_ROS_RT_PUBLISHER

class RealTimePublisherJoints
{
	public:
		
		/** Initialize the real time publisher. */
		RealTimePublisherJoints(const ros::NodeHandle& ros_nh, const std::string topic_name, int msg_size, Eigen::VectorXd init_cond)
		{
			// Checks
			assert(msg_size > 0);
			assert(topic_name.size() > 0);
			
			topic_name_ = topic_name;
			
			assert(init_cond.size() >=  msg_size);
			msg_size_ = msg_size;
			
			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
			for(int i = 0; i < msg_size_; i++){
				pub_ptr_->msg_.name.push_back("joint_"+std::to_string(i));
				pub_ptr_->msg_.position.push_back(init_cond[i]);
				
				//pub_ptr_->msg_.position.push_back(0.0);
				//pub_ptr->msg_.velocity.push_back(0.0);
				//pub_ptr->msg_.effort.push_back(0.0);
			}
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();
				for(int i = 0; i < msg_size_; i++)
				{
					pub_ptr_->msg_.position[i] = in[i];
				}
				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<sensor_msgs::JointState> rt_publisher_t;
		std::string topic_name_;
		int msg_size_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

class RealTimePublisherPath
{
	public:
		
		/** Initialize the real time publisher. */
		RealTimePublisherPath(const ros::NodeHandle& ros_nh, const std::string topic_name, int msg_size, Eigen::VectorXd init_cond)
		{
			// Checks
			assert(msg_size > 0);
			assert(topic_name.size() > 0);
			
			topic_name_ = topic_name;
			
			assert(init_cond.size() >= 3);
			prev_pose_.pose.position.x = init_cond[0];
			prev_pose_.pose.position.y = init_cond[1];
			prev_pose_.pose.position.z = init_cond[2];
			
			//msg_size_ = 2; // NOTE msg_size_ is usless...
			//geometry_msgs::PoseStamped empty_pose;

			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
			pub_ptr_->msg_.header.frame_id = "T0"; // HACK
			for(int i = 0; i < 2; i++){ // NOTE For a line we need two points
				pub_ptr_->msg_.poses.push_back(prev_pose_);
			}
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();
				//for(int i = 0; i < msg_size_; i++)
				//{
				pub_ptr_->msg_.poses[0] = prev_pose_;
				
				pub_ptr_->msg_.poses[1].pose.position.x = in[0];
				pub_ptr_->msg_.poses[1].pose.position.y = in[1];
				pub_ptr_->msg_.poses[1].pose.position.z = in[2];
				
				prev_pose_ = pub_ptr_->msg_.poses[1];
				
				//}
				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<nav_msgs::Path> rt_publisher_t;
		std::string topic_name_;
		//int msg_size_;
		geometry_msgs::PoseStamped prev_pose_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

class RealTimePublisherWrench
{
	public:
		
		/** Initialize the real time publisher. */
		RealTimePublisherWrench(const ros::NodeHandle& ros_nh, const std::string topic_name, std::string frame_id)
		{
			// Checks
			//assert(msg_size > 0);
			assert(topic_name.size() > 0);
			
			topic_name_ = topic_name;
			
			//assert(init_cond.size() >= 3);
			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
			
// 			pub_ptr_->msg_.wrench.force.x = init_cond[0];
// 			pub_ptr_->msg_.wrench.force.y = init_cond[1];
// 			pub_ptr_->msg_.wrench.force.z = init_cond[2];
// 			pub_ptr_->msg_.wrench.torque.x = init_cond[3];
// 			pub_ptr_->msg_.wrench.torque.y = init_cond[4];
// 			pub_ptr_->msg_.wrench.torque.z = init_cond[5];
			
			
			pub_ptr_->msg_.header.frame_id = frame_id;
			
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();

				// Publish the force
				pub_ptr_->msg_.wrench.force.x = in[0];
				pub_ptr_->msg_.wrench.force.y = in[1];
				pub_ptr_->msg_.wrench.force.z = in[2];
				
				// Publish the torques
				if(in.size() > 3) 
				{
				  pub_ptr_->msg_.wrench.torque.x = in[3];
				  pub_ptr_->msg_.wrench.torque.y = in[4];
				  pub_ptr_->msg_.wrench.torque.z = in[5];
				}
				
				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> rt_publisher_t;
		std::string topic_name_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

class RealTimePublisherMarkers
{
	public:
		
		/** Initialize the real time publisher. */
		RealTimePublisherMarkers(const ros::NodeHandle& ros_nh, const std::string topic_name, std::string frame_id)
		{
			// Checks
			//assert(msg_size > 0);
			assert(topic_name.size() > 0);
			
			topic_name_ = topic_name;
			
			//assert(init_cond.size() >= 3);
			pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));
			
			pub_ptr_->msg_.header.frame_id = frame_id;
			
			pub_ptr_->msg_.type = visualization_msgs::Marker::SPHERE; // FIXME Hardcored sphere
			pub_ptr_->msg_.action = visualization_msgs::Marker::ADD;
			pub_ptr_->msg_.ns = "marker";
			pub_ptr_->msg_.id = 0;
			
			pub_ptr_->msg_.color.r = 0.0;
			pub_ptr_->msg_.color.g = 1.0;
			pub_ptr_->msg_.color.b = 0.0;
	  
			//marker.pose.orientation.x = 0.0; // FIXME I can use these infos to add the covariance (rotation)
			//marker.pose.orientation.y = 0.0;
			//marker.pose.orientation.z = 0.0;
			pub_ptr_->msg_.pose.orientation.w = 1.0;
			  
		}
		/** Publish the topic. */
		inline void publish(const Eigen::Ref<const Eigen::VectorXd>& in)
		{
			if(pub_ptr_ && pub_ptr_->trylock())
			{ 
				pub_ptr_->msg_.header.stamp = ros::Time::now();

				// Mean
				pub_ptr_->msg_.pose.position.x = in[0];
				pub_ptr_->msg_.pose.position.y = in[1];
				pub_ptr_->msg_.pose.position.z = in[2];
				
				// Variance
				pub_ptr_->msg_.scale.x = in[3];
				pub_ptr_->msg_.scale.y = in[4];
				pub_ptr_->msg_.scale.z = in[5];

				pub_ptr_->unlockAndPublish();
			}
		}
		
		inline std::string getTopic(){return topic_name_;}
		
	private:
		
		typedef realtime_tools::RealtimePublisher<visualization_msgs::Marker> rt_publisher_t;
		std::string topic_name_;
		boost::shared_ptr<rt_publisher_t > pub_ptr_;
		//visualization_msgs::Marker marker_;
};


//typedef std::map<std::string,std::pair<Eigen::VectorXd*,RealTimePublisher_t*> > pubs_map_t;
//typedef pubs_map_t::iterator pubs_map_it_t;

template <class RealTimePublisher_t>
class RealTimePublishers
{
	public:

		RealTimePublishers(){};
// 		~RealTimePublishers()
// 		{
// 			for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
// 				delete iterator->second.second;
// 		}
		
		// Add a RealTimePublisher already created
		void AddPublisher(boost::shared_ptr<RealTimePublisher_t> pub_ptr, Eigen::VectorXd* vector_ptr) 
		{
			assert(pub_ptr!=false);
			// Put it into the map with his friend
			map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
		}
		
		// Add a new fresh RealTimePublisher
		void AddPublisher(const ros::NodeHandle& ros_nh, const std::string topic_name, int msg_size, Eigen::VectorXd* vector_ptr, Eigen::VectorXd init_cond = Eigen::VectorXd::Zero(50)) //HACK 50 because it's rare to have more then 50 dofs...
		{
			boost::shared_ptr<RealTimePublisher_t> pub_ptr = boost::make_shared<RealTimePublisher_t>(ros_nh,topic_name,msg_size,init_cond);
			assert(pub_ptr!=false);
			// Put it into the map with his friend
			map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
		}
		void PublishAll()
		{
			for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
				iterator->second.second->publish(*iterator->second.first);
		}
	private:
		
		//typedef std::map<std::string,std::pair<Eigen::VectorXd*,RealTimePublisher_t*> > pubs_map_t;
		typedef std::map<std::string,std::pair<Eigen::VectorXd*,boost::shared_ptr<RealTimePublisher_t> > > pubs_map_t;
		typedef typename pubs_map_t::iterator pubs_map_it_t;
		
		pubs_map_t map_;
};
#endif

template<typename value_t>
void WrapRad(value_t &rad)
{
	while (rad >= M_PI){
		rad -= 2*M_PI;
	}
	while (rad <= -M_PI){
		rad += 2*M_PI;
	}
}

template<typename value_t>
void UnwrapRad(value_t &rad, value_t max, value_t min)
{
	if (!((rad > min) && (rad < max))){
		if (rad + 2*M_PI < max)
			rad = rad + 2*M_PI;
		else if (rad - 2*M_PI > min)
			rad = rad - 2*M_PI;
	}
}



}
		
#endif

