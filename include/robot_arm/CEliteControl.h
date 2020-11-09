//
// Created by zhoujg on 19-12-6.
//

#ifndef PROJECT_CELITECONTROL_H
#define PROJECT_CELITECONTROL_H
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <deque>
#include <mutex>
#include <thread>
#include <queue>
#include <time.h>
#include <condition_variable>
#include <cmath>
#include <fcntl.h>
#include <math.h>
#include <fstream>
#include <unistd.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <jsoncpp/json/json.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include "elt_ctx.h"
#include "elt_robot.h"
#include "elt_rpc.h"
#include "cJSON.h"
#include "elt_transport.h"
#include "elt_define.h"
#include "wootion_msgs/GeneralCmd.h"
#include "wootion_msgs/GeneralAck.h"
#include "wootion_msgs/ControlService.h"
#include "wootion_msgs/GeneralTopic.h"
#include "wootion_msgs/RobotStatus.h"
#include "wootion_msgs/GeneralAction.h"
#include "CFileRW.h"

using namespace std;

typedef enum _tagEltStatus
{
    STOP = 0,
    PAUSE = 1,
    EMERGENCY_STOP = 2,
    MOVING = 3,
    ALARM = 4,
    DROP_LINE = 5,
}EltStatus;

typedef enum _tagArmStatus
{
    IDLE = 0,
    RECORD = 1,
    PLAY = 2,
    RESET = 3,
    ROTATE = 4,
    TURN_AROUND = 5,
    SET_ARM_YAW = 6,
    TEACH = 7,
    ERROR = 8,
    SET_ORIENTATION = 9,
}ArmStatus;

const static vector<string> vsArmStatus = {
    "idle",
    "record",
    "play",
    "reset",
    "rotate",
    "turn_around",
    "set_arm_yaw",
    "teach",
    "error",
    "set_orientation",
};

typedef enum _tagDragStatus
{
    DISABLE,
    ENABLE,
    FORWARD,
    REVERSE
}DragStatus;

typedef struct _tagEltPos
{
    elt_robot_pos eltPos={0};
} EltPos, *PEltPos;

typedef enum _tagAxisLimit
{
    AXIS_ONE_MAX,
    AXIS_TWO_MAX,
    AXIS_THREE_MAX,
    AXIS_FOUR_MAX,
    AXIS_FIVE_MAX,
    AXIS_SIX_MAX,
    
    AXIS_ONE_MIN,
    AXIS_TWO_MIN,
    AXIS_THREE_MIN,
    AXIS_FOUR_MIN,
    AXIS_FIVE_MIN,
    AXIS_SIX_MIN,
}AxisLimit;

typedef enum _tagAxisNum
{
    AXIS_ONE,
    AXIS_TWO,
    AXIS_THREE,
    AXIS_FOUR,
    AXIS_FIVE,
    AXIS_SIX,
}AxisNum;

typedef enum _tagRotateType
{
    AXIS_PITCH = AXIS_FOUR,
    AXIS_YAW = AXIS_FIVE,
    AXIS_ROLL = AXIS_SIX,
}RotateType;

/*name      max        min   度(°)

1	177.0	-177.0
2	177.0	-177
3	160.0	-160.0
4	80.0	-260.0
5	357.0	 3.0
6	177.0	-177.0*/

//大
const static double AxisLimitAngle[12] = {
    358.0,  358.0,  158.0,  358.0,  358.0,  358.0,
   -358.0, -358.0, -158.0, -358.0, -358.0, -358.0
};

typedef struct _tagResetIni
{
	string sOrbitFile;
	int nPlayFirstAxis;
	int nValid;
} ResetIni, *PResetIni;

class CEliteControl
{
public:
    CEliteControl();
    ~CEliteControl();

    void UnInit();
    void UpdateEliteThreadFunc();
    void HeartBeatThreadFunc();
    void MonitorThreadFunc();
    void ArmCmdCallBack(const wootion_msgs::GeneralCmd::ConstPtr &TerraceCmd);
    void AgvStatusCallBack(const wootion_msgs::RobotStatus::ConstPtr &AgvStatus);
    void LinearSmooth7(double *dSrc, double *dDest, int nLen);
    void PrintJointData(elt_robot_pos &pos_array, string sFunName);
    void RemoveErrPoints(deque<EltPos> &trackDeque);
    void RemoveOverduePoints(deque<EltPos> &trackDeque);

    int UpdateEltOrigin();
    int EliteJointMove(elt_robot_pos &targetPos, double dSpeed, string &sErr);
    int EliteMultiPointMove(elt_robot_pos &targetPos, double dSpeed, string &sErrMsg, int nType=0);
    int GetElitePos(elt_robot_pos &pos_array);
    int EliteStop(string &sErr);
    int EliteDrag(int nCmd);
    int EliteRunDragTrack(const string &sFileName, int nPlayFirstAxis, double dSpeed, int nDirection, string &sErrMsg);
    int EliteSyncMotorStatus(bool bSwitchStatusFirst=true);
    int EliteClearAlarm();
    int EliteOpenServo();
    int CreateDataPath();

    bool Init();
    bool ResetToOrigin(string &sOutput);
    bool EliteGotoOrigin(string &sOutput);
    bool CheckOrigin(elt_robot_pos &pos_array, bool bCheckBase = false);
    bool WaitForMotionStop(int nTimeoutTime, string &sErr);
    bool ArmServiceFunc(wootion_msgs::ControlService::Request &Req, wootion_msgs::ControlService::Response &Resp);
    bool ArmOperation(const std::string &sCommand, const std::string &sInput, std::string &sOutput);
    bool RecordOrbit(const std::string &sInput, std::string &sOutput);
    bool StopOrbit(std::string &sOutput);
    bool PlayOrbit(const std::string &sInput, std::string &sOutput);
    bool Rotate(const std::string &sInput, std::string &sOutput);
    bool StopRotate(std::string &sOutput);
    bool GetOrientation(std::string &sOutput);
    bool SetOrientation(const std::string &sInput, std::string &sOutput);
    bool GetPosition(std::string &sOutput);
    bool SetPosition(const std::string &sInput, std::string &sOutput);
    bool Reset(std::string &sOutput);
    bool GetYawAngle(std::string &sOutput);
    bool SetYawAngle(const std::string &sInput, std::string &sOutput);
    bool TurnAround(std::string &sOutput);
    bool GotoNewPos(const std::string &sInput, std::string &sOutput);
    bool VirtualTeach(const std::string &sInput, std::string &sOutput);
	bool RecordResetInfoToIni(const string sFileName, const ResetIni resetInfo);
	bool GetResetInfoFromIni(const string sFileName, ResetIni &resetInfo);
	bool GetOrbitEndPoint(const string sFileName, elt_robot_pos &EndPoint, std::string &sOutput);

private:
    string m_sNodeName;
    string m_sEliteRobotIP;
    string m_sPrintLevel;
    string m_sArmTrackPath;
    string m_sArmService;
    string m_sArmCmdTopic;
    string m_sArmAckTopic;
    string m_sHeartBeatTopic;
    string m_sAbnormalTopic;
    string m_sOrbitFileName;
    string m_sResetIniFile;
    string m_sStatus;
    string m_sAgvStatusTopic;
    string m_sArmOrigin;
    string m_sOrbitGroup;
    string m_sOrbitSecurity;
    string m_sAntennaConflictZone1;
    string m_sAntennaConflictZone2;

    int m_nElitePort;
    int m_nEliteState;
    int m_nCheckReceiver;
    int m_nDragStatus;
    int m_nTaskName;
    int m_nTimeoutLen;
    int m_nEliteMode;
    int m_nSmoothnessLevel;
    int m_nDebugTeach;
    int m_nPlayType;

    bool m_bRecordDragTrack;
    bool m_bBusy;
    bool m_bEmeStop;
    bool m_bArmInit;
    bool m_bResetFromNearestPoint;
    bool m_bDragRecord;

    double m_dEltSpeed;//百分比
    double m_dRotateSpeed;//百分比
    double m_dRotateLimitAngle;
    double m_dOrbitStep;
    double m_dFilterValue;
    double m_dMultiPointStep;
    double m_dRobotOrientation;
    double m_dRobotPositionX;
    double m_dRobotPositionY;
    double m_dConflictZone1Axis2;
    double m_dConflictZone2Axis2;

    timespec m_tRecordDataTime;

    ELT_CTX m_eltCtx;
    elt_robot_pos m_EliteCurrentPos;
    elt_robot_pos m_RotateOriginPos;
    elt_robot_pos m_EltOriginPos;
    std::deque<EltPos> m_dDragTrackDeque;

    std::thread *m_pMonitorThread;
    std::thread *m_pEliteStatusThread;
    std::thread *m_pHeartBeatThread;

    ros::Subscriber m_ArmCmdSubscriber;
    ros::Subscriber m_AgvStatusSubscriber;
    ros::Publisher m_ArmAckPublisher;
    ros::Publisher m_HeartBeatPublisher;
    ros::Publisher m_AbnormalPublisher;

    ros::ServiceServer m_ArmService;

    CFileRW m_TrackFile;

	ResetIni m_ResetInfo;

	std::vector<double> m_vdConflictZone1Axis1;
	std::vector<double> m_vdConflictZone2Axis1;
};

#endif //PROJECT_CELITECONTROL_H
