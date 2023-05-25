#include "imu_gps_localizer/imu_gps_localizer.h"

#include <glog/logging.h>

#include <iostream>

#include <iomanip>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                 const double acc_bias_noise, const double gyro_bias_noise,
                                 const Eigen::Vector3d& I_p_Gps) 
    : initialized_(false){
    initializer_ = std::make_unique<Initializer>(I_p_Gps);
    imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, 
                                                    acc_bias_noise, gyro_bias_noise,
                                                    Eigen::Vector3d(0., 0., -9.81007));
    gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) {
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }
    
    // Predict.
    imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

    // Convert ENU state to lla.
    ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
    *fused_state = state_;
    return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr) {
    if (!initialized_) {
        if (!initializer_->AddGpsPositionData(gps_data_ptr, &state_)) {
            return false;
        }

        // Initialize the initial gps point used to convert lla to ENU.
        Eigen::Vector3d set_lla;
        //3.bag:
        //set_lla(0) = 31.6495779128333;
        //set_lla(1) = 121.590637205333;
        set_lla(0) = 31.6495775156667;
        set_lla(1) = 121.5906373801667;
        set_lla(2) = 14.6325;
        //init_lla_ = gps_data_ptr->lla;
        init_lla_ = set_lla;
        std::cout << "init_lla_ = \n" << std::setprecision(15) << init_lla_.transpose() << std::endl;
        initialized_ = true;

        LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }

    // Update.
    gps_processor_->UpdateStateByGpsPosition(init_lla_, gps_data_ptr, &state_);

    return true;
}

}  // namespace ImuGpsLocalization