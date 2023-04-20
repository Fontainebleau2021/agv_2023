#ifndef NAVI_STATE_H
#define NAVI_STATE_H

#include <memory>

namespace navigation
{
    
    class NaviMasterRos;
    class NaviState
    {
    public:
        NaviState() {}
        ~NaviState() {}
        virtual void Excute(NaviMasterRos* navi_master) = 0;
        void StateChange(NaviMasterRos* navi_master);
    };

    //初始化
    class InitState : public NaviState
    {
    public:
        ~InitState() {}
        virtual void Excute(NaviMasterRos* navi_master); 
    };

    //等待指令
    class WaitState : public NaviState
    {
    public:
        ~WaitState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };

    //初始规划
    class InitPlanState : public NaviState
    {
    public:
        ~InitPlanState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };



    //自主规划的速度控制状态
    class NaviControlState : public NaviState
    {
    public:
        ~NaviControlState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };

    //Stop
    class StopState : public NaviState
    {
    public:
        ~StopState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };


    //播种直线控制的状态
    class SeedLineControlState : public NaviState
    {
    public:
        ~SeedLineControlState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };

    //播种转弯控制的状态
    class SeedTurnControlState : public NaviState
    {
    public:
        ~SeedTurnControlState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };

    //直行
    class StratControlState : public NaviState
    {
    public:
        ~StratControlState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };

    //转弯
    class TurnControlState : public NaviState
    {
    public:
        ~TurnControlState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };

    //目标点的播种控制
    class PIDSeedControlState : public NaviState
    {
    public:
        ~PIDSeedControlState() {}
        virtual void Excute(NaviMasterRos* navi_master);
    };

    typedef std::shared_ptr<NaviState> NaviStatePtr;

} // namespace navigation

#endif // NAVI_STATE_H