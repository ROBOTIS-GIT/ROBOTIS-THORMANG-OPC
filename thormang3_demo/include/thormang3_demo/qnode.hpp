/**
 * @file /include/thor3_control/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef thor3_control_QNODE_HPP_
#define thor3_control_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sstream>

#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"

#include "thormang3_base_module_msgs/BothWrench.h"
#include "thormang3_base_module_msgs/CalibrationWrench.h"

// manipulation demo
#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/DemoPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_manipulation_module_msgs/GetJointPose.h"
#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

// walking demo
#include "thormang3_foot_step_generator/FootStepCommand.h"
#include <std_msgs/Bool.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thor3_control {

/*****************************************************************************
** Class
*****************************************************************************/

class QNodeThor3 : public QThread {
        Q_OBJECT
    public:
        QNodeThor3(int argc, char** argv );
        virtual ~QNodeThor3();
        bool init();
        void run();

        /*********************
    ** Logging
    **********************/
        enum LogLevel {
            Debug,
            Info,
            Warn,
            Error,
            Fatal
        };

        enum CONTROL_INDEX
        {
            Control_None = 0,
            Control_Walking = 1,
            Control_Manipulation = 2,
            Control_Head = 3,
        };

        std::map< int, std::string > preset_index;
        std::map< std::string, std::vector<int> > preset_table;

        QStringListModel* loggingModel() { return &logging_model_; }
        void log( const LogLevel &level, const std::string &msg);
        void clearLog();
        void assemble_lidar();
        void setJointControlMode(const robotis_controller_msgs::JointCtrlModule &msg);
        bool getJointNameFromID(const int &id, std::string &joint_name);
        bool getIDFromJointName(const std::string &joint_name, int &id);
        bool getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name);
        std::string getModeName(const int &index);
        int getModeIndex(const std::string &mode_name);
        int getModeSize();
        int getJointSize();
        void clearUsingModule();
        bool isUsingModule(std::string module_name);
        void moveInitPose();
        void initFTCommand(std::string command);

        void setHeadJoint(double pan, double tilt);

        // Manipulation
        void sendInitPoseMsg( std_msgs::String msg );
        void sendDestJointMsg( thormang3_manipulation_module_msgs::JointPose msg );
        void sendIkMsg( thormang3_manipulation_module_msgs::KinematicsPose msg );
        void sendDemoMsg( thormang3_manipulation_module_msgs::DemoPose msg ) ;

        // Walking
        void setWalkingCommand( thormang3_foot_step_generator::FootStepCommand msg);
        void setWalkingBalance(bool on_command);

    public Q_SLOTS:
        void getJointControlMode();
        void getJointPose( std::string joint_name );
        void getKinematicsPose (std::string group_name );
        void getKinematicsPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);

    Q_SIGNALS:
        void loggingUpdated();
        void rosShutdown();
        void updateCurrentJointControlMode(std::vector<int> mode);

        // Manipulation
        void updateCurrJoint( double value );
        void updateCurrPos( double x , double y , double z );
        void updateCurrOri( double x , double y , double z , double w );

        void updateHeadAngles(double pan, double tilt);

    private:
        int init_argc;
        char** init_argv;
        const bool DEBUG;
        ros::Publisher init_pose_pub_;
        ros::Publisher init_ft_pub_;
        ros::Publisher module_control_pub_;
        ros::Subscriber init_ft_foot_sub_;
        ros::Subscriber both_ft_foot_sub_;
        ros::Subscriber current_module_control_sub_;
        ros::ServiceClient get_module_control_client_;

        // Head
        ros::Publisher move_lidar_pub_;
        ros::Publisher set_head_joint_angle_pub_;
        ros::Subscriber current_joint_states_sub_;

        // Manipulation
        ros::Publisher set_control_mode_msg_pub;
        ros::Publisher send_ini_pose_msg_pub;
        ros::Publisher send_des_joint_msg_pub;
        ros::Publisher send_ik_msg_pub;
        ros::Publisher send_pathplan_demo_pub;
        ros::Subscriber kenematics_pose_sub;
        ros::ServiceClient get_joint_pose_client;
        ros::ServiceClient get_kinematics_pose_client;

        // Walking
        ros::Publisher set_walking_command_pub;
        ros::Publisher set_walking_balance_pub;

        ros::Time start_time_;

        QStringListModel logging_model_;
        std::map<int, std::string> id_joint_table_;
        std::map<std::string, int> joint_id_table_;

        std::map<int, std::string> index_mode_table_;
        std::map<std::string, int> mode_index_table_;

        std::map<std::string, bool> using_mode_table_;


        void parseJointNameFromYaml(const std::string &path);
        void refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);
        void updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
        void initFTFootCallback(const thormang3_base_module_msgs::BothWrench::ConstPtr &msg);
        void calibrationFTFootCallback(const thormang3_base_module_msgs::CalibrationWrench::ConstPtr &msg);
};

}  // namespace thor3_control

#endif /* thor3_control_QNODE_HPP_ */
