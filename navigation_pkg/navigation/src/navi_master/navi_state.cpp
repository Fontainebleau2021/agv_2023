#include "navi_state.h"
#include "navigation_ros.h"
#include <chrono>

namespace navigation
{
    int num_dir = 0;
    int num_idx = 0;
    int seed_x = 1;
    int seed_y = 1;
    int theta_heading = -1;
    double heading = M_PI_2;
    bool rot_finish = true;

    void NaviState::StateChange(NaviMasterRos *navi_master)
    {
        // navi_master->in_control_ = false;
        switch (navi_master->NaviCmd())
        {
        case NaviCommand::WaitCmd:
            navi_master->SetNaviState(std::make_shared<WaitState>());
            //ROS_INFO("waiting!!!navi_master->in_seed_:%d",navi_master->in_seed_);
            //ROS_INFO("waiting!!!navi_master->seed_run:%d",navi_master->seed_run);
            // if (navi_master->in_seed_ == false)
            // {
            //     seed_x = 1;
            //     seed_y = 1;
            //     theta_heading = 0;
            //     navi_master->seed_run = false;
            //     navi_master->SetNaviState(std::make_shared<WaitState>());
            // }
            // else
            // {
            //     if (navi_master->seed_run == false)
            //     {
            //         navi_master->SetNaviState(std::make_shared<PIDSeedControlState>());
            //         ROS_INFO("pubing------------------");
            //     }
            //     else
            //     {
            //         navi_master->SetNaviState(std::make_shared<WaitState>());
            //     }
            // }
            break;
        case NaviCommand::NaviPlanCmd:
            navi_master->SetNaviState(std::make_shared<InitPlanState>());
            break;
        case NaviCommand::NaviControlCmd:
            navi_master->SetNaviState(std::make_shared<NaviControlState>());
            break;
        case NaviCommand::ReInitCmd:
            navi_master->SetNaviState(std::make_shared<InitPlanState>());
            break;
        case NaviCommand::StopCmd:
            navi_master->SetNaviState(std::make_shared<StopState>());
            navi_master->seed_run = false;
            break;
        case NaviCommand::LineControlCmd:
            navi_master->UpdateLineStartPose();
            navi_master->in_control_ = true;
            navi_master->SetNaviState(std::make_shared<SeedLineControlState>());
            break;
        case NaviCommand::RotateControlCmd:
            navi_master->SetNaviState(std::make_shared<SeedTurnControlState>());
            break;
        case NaviCommand::StratCmd:
            navi_master->SetNaviState(std::make_shared<StratControlState>());
            break;
        case NaviCommand::TurnCmd:
            navi_master->SetNaviState(std::make_shared<TurnControlState>());
            break;
        case NaviCommand::PIDseed:
            if(navi_master->in_seed_ == false)
            {
                navi_master->in_seed_ = true;
                navi_master->SetNaviState(std::make_shared<PIDSeedControlState>());
            }
            else
            {
                navi_master->SetNaviState(std::make_shared<WaitState>());
            }    
            //ROS_INFO("PUBING!!!navi_master->in_seed_:%d",navi_master->in_seed_);
            //ROS_INFO("PUBING!!!navi_master->seed_run:%d",navi_master->seed_run);
            break;
        default:
            break;
        }
    }

    void InitState::Excute(NaviMasterRos *navi_master)
    {

        navi_master->Init();
        ROS_INFO("Init finish!");
        navi_master->SetNaviCmd(NaviCommand::WaitCmd);
    }

    void WaitState::Excute(NaviMasterRos *navi_master)
    {
        ROS_INFO(" Now robot is waiting cmd! ");
        // 这个是可视化
        navi_master->Visualize(VisualizeType::COSTMAP);
        navi_master->SeedLineVisualize();
        //navi_master->SetNaviCmd(NaviCommand::WaitCmd);
    }

    void InitPlanState::Excute(NaviMasterRos *navi_master)
    {

        navi_master->Visualize(VisualizeType::COSTMAP);
        std::chrono::steady_clock::time_point s_time = std::chrono::steady_clock::now();
        bool success = navi_master->GlobalPlan();
        if (success)
        {
            success = navi_master->LocalInitPlan();
            auto e_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = e_time - s_time;
            std::cout << "time: " << elapsed.count() << "s" << std::endl;
            if (success)
            {
                ROS_INFO("Plan success, into control");
                navi_master->Visualize(VisualizeType::PLAN);
                navi_master->SetNaviCmd(NaviCommand::NaviControlCmd);
                return;
            }
        }
        ROS_INFO("Plan failed, continue waiting");
        navi_master->SetNaviCmd(NaviCommand::WaitCmd);
    }

    void NaviControlState::Excute(NaviMasterRos *navi_master)
    {
        navi_master->GlobalPlan();
        if (navi_master->IsNaviArriveTarget())
        {
            ROS_INFO(" We have arrive at target! ");
            navi_master->SetNaviCmd(NaviCommand::StopCmd);
            return;
        }
        navi_master->GetControlVelocity();
        ROS_INFO(" Now is in navigation control");
    }

    void StopState::Excute(NaviMasterRos *navi_master)
    {
        navi_master->Stop();
        navi_master->SetNaviCmd(NaviCommand::WaitCmd);
    }

    void SeedLineControlState::Excute(NaviMasterRos *navi_master)
    {
        if (navi_master->SeedIsLineArriveTerminal(num_dir, num_idx))
        {
            ROS_INFO(" We have finish seed on one line! ");
            if ((num_dir == 1)&&(num_idx == 10))
            {
                navi_master->in_control_ = false;
                navi_master->SetNaviCmd(NaviCommand::StopCmd);
            }
            else
            {
                navi_master->SetNaviCmd(NaviCommand::RotateControlCmd);
            }
            return;
        }
        navi_master->LineControl(heading);
    }

    void SeedTurnControlState::Excute(NaviMasterRos *navi_master)
    {
        if ((num_dir == 0)&&(num_idx == 0))
        {
            if (navi_master->SeedIsRotateArriveTerminal(0))
            {
                ROS_INFO(" We have finish rotate! ");
                num_dir = 1;
                heading = 0.0;
                navi_master->SetNaviCmd(NaviCommand::LineControlCmd);
                return;
            }
            navi_master->RotateControl(RotateTargetTheta[0]);
        }
        else
        {
            if ((num_dir == 1)&&(num_idx == 0))
            {
                if (navi_master->SeedIsRotateArriveTerminal(1))
                {
                    ROS_INFO(" We have finish rotate! ");
                    num_dir = 0;
                    num_idx = 1;
                    heading = M_PI_2;
                    navi_master->SetNaviCmd(NaviCommand::LineControlCmd);
                    return;
                }
                navi_master->RotateControl(RotateTargetTheta[1]);
            }
            else
            {
                if (num_dir == 0) 
                {
                    if (num_idx%2 == 1)
                    {
                        if (navi_master->SeedIsRotateArriveTerminal(0))
                        {
                            ROS_INFO(" We have finish rotate! ");
                            num_dir = 1;
                            heading  = 0.0;
                            navi_master->SetNaviCmd(NaviCommand::LineControlCmd);
                            return;
                        }
                        navi_master->RotateControl(RotateTargetTheta[0]);
                    }
                    else
                    {
                        if (navi_master->SeedIsRotateArriveTerminal(2))
                        {
                            ROS_INFO(" We have finish rotate! ");
                            num_dir = 1;
                            heading  = M_PI;
                            navi_master->SetNaviCmd(NaviCommand::LineControlCmd);
                            return;
                        }
                        navi_master->RotateControl(RotateTargetTheta[2]);
                    }
                }
                else
                {
                    if (num_idx%2 == 1)
                    {
                        if (navi_master->SeedIsRotateArriveTerminal(3))
                        {
                            ROS_INFO(" We have finish rotate! ");
                            num_dir = 0;
                            num_idx = num_idx + 1;
                            heading  = -M_PI_2;
                            navi_master->SetNaviCmd(NaviCommand::LineControlCmd);
                            return;
                        }
                        navi_master->RotateControl(RotateTargetTheta[3]);
                    }
                    else
                    {
                        if (navi_master->SeedIsRotateArriveTerminal(1))
                        {
                            ROS_INFO(" We have finish rotate! ");
                            num_dir = 0;
                            num_idx = num_idx + 1;
                            heading  = M_PI_2;
                            navi_master->SetNaviCmd(NaviCommand::LineControlCmd);
                            return;
                        }
                        navi_master->RotateControl(RotateTargetTheta[1]);
                    }
                }

            }
        }
        
    }

    void StratControlState::Excute(NaviMasterRos *navi_master)
    {
        // if(navi_master->IsLineArriveTerminal(0,0))
        // {
        //     ROS_INFO(" We have finish seed on one line! ");
        //     navi_master->in_control_ = false;
        //     navi_master->SetNaviCmd(NaviCommand::StopCmd);
        //     return;
        // }
        // navi_master->LineControl(heading);
        navi_master->PubPIDGoal(74,0,0);
    }

    void TurnControlState::Excute(NaviMasterRos *navi_master)
    {
        if(navi_master->IsRotateArriveTerminal(0))
        {
            ROS_INFO(" We have finish rotate! ");
            navi_master->SetNaviCmd(NaviCommand::StopCmd);
            return;
        }
        navi_master->RotateControl(RotateTargetTheta[0]);
    }

    void PIDSeedControlState::Excute(NaviMasterRos *navi_master)
    {
        if(rot_finish == true)
        {
            navi_master->PubPIDGoal(74,0,0);
        }
        
        
    }

} // namespace navigation
