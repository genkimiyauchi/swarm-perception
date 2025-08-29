#ifndef TARGET_TRACKING_WEBVIZUSER_FUNCTIONS_H
#define TARGET_TRACKING_WEBVIZUSER_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/visualizations/webviz/webviz_user_functions.h>

#include <utility/robot_message.h>

#include <loop_functions/target_tracking_loop_functions/target_tracking_loop_functions.h>

#include <controllers/robot/robot.h>

#include <iostream>
#include <fstream>

using namespace argos;

class CTargetTrackingWebvizUserFunctions : public CWebvizUserFunctions {
    public:
        CTargetTrackingWebvizUserFunctions();

        virtual ~CTargetTrackingWebvizUserFunctions();

        virtual void HandleCommandFromClient(const std::string& str_ip, nlohmann::json c_json_command);

        virtual const nlohmann::json sendUserData();
        
        virtual const nlohmann::json sendRobotData(CEPuckEntity& robot);

        virtual void ClientConnected(std::string str_id);

        virtual void ClientDisconnected(std::string str_id);

    private:

        CTargetTrackingLoopFunctions *m_pcExperimentLoopFunctions;

        struct ClientData {
            std::string id = "";
            std::string username = "";
        };

        /*
         * Map storing the ws pointer to client ID.
         * Key is the client pointer. Value is client ID.
         */
        std::map<std::string, ClientData> m_pcClientPointerToId;

        /* 
         * Map of connections between robots and clients 
         * Key is the robot ID. Value is client ID.
         */
        std::map<std::string, ClientData> m_pcClientRobotConnections;

        // /*
        //  * Map storing each client's last move command.
        //  * Key is the user ID. Value is the direction.
        //  */
        // std::map<std::string, std::string> m_pcLastClientMoveCommands;

        char m_cLastDirection = 'U'; // default to move forward
        bool m_bLogging;
        std::string m_strCommandFilePath;
        std::ofstream m_cOutput;
};

#endif