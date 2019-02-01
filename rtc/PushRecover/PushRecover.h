// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
/*!
 * @file  PushRecover.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PUSHRECOVER_H
#define PUSHRECOVER_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/EigenTypes.h>

#include "../ImpedanceController/RatsMatrix.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../TorqueFilter/IIRFilter.h"

#if defined(__INTEL_COMPILER)||defined(__ICC)
#include "QzMatrix.h"
#include "Vec3e.h"
#endif
#if defined(__GNUC__)
#include "link_kinematics.h"
#include "Vec3e.h"
#else
#error "hoge"
#endif

#include "BodyIKMethod.h"
#include "ReactivePatternGenerator.h"
#include "OnlinePatternGenerator.h"
#include "RobustOnlinePatternGenerator.h"
#define USE_APECF
#if defined(USE_APECF)
#include "AbsolutePoseEstimator_CF.h"
#else
#include "AbsolutePoseEstimator.h"
#endif
#include "interpolator.h"  /* from hrpsys/rtc/SequencePlayer */
#include "FrameRateMatcher.h"
#include "SimpleLogger.h"
#include <algorithm>
#include <boost/shared_ptr.hpp>

/* for gettimeofday */
#include <sys/time.h>
#include <time.h>
#include "WheelController.h"

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "PushRecoverService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

#define DEBUGP(x) ((m_debugLevel==1 && loop%x==0) || m_debugLevel > 1 )
#define USE_ROBUST_ONLINE_PATTERN_GENERATOR

/**
   \brief sample RT component which has one data input port and one data output port
*/
class PushRecover
    : public RTC::DataFlowComponentBase
{
public:
    /**
       \brief Constructor
       \param manager pointer to the Manager
    */
    PushRecover(RTC::Manager* manager);
    /**
       \brief Destructor
    */
    virtual ~PushRecover();

    // The initialize action (on CREATED->ALIVE transition)
    // formaer rtc_init_entry()
    virtual RTC::ReturnCode_t onInitialize();

    // The finalize action (on ALIVE->END transition)
    // formaer rtc_exiting_entry()
    virtual RTC::ReturnCode_t onFinalize();

    // The startup action when ExecutionContext startup
    // former rtc_starting_entry()
    // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

    // The shutdown action when ExecutionContext stop
    // former rtc_stopping_entry()
    // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

    // The activated action (Active state entry action)
    // former rtc_active_entry()
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

    // The deactivated action (Active state exit action)
    // former rtc_active_exit()
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

    // The execution action that is invoked periodically
    // former rtc_active_do()
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    // The aborting action when main logic error occurred.
    // former rtc_aborting_entry()
    // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

    // The error action in ERROR state
    // former rtc_error_do()
    // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

    // The reset action that is invoked resetting
    // This is same but different the former rtc_init_entry()
    // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

    // The state update action that is invoked after onExecute() action
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

    // The action that is invoked when execution context's rate is changed
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);
    //bool setTendonPairParam(const PushRecoverService::TendonPairParam& i_param);
    //bool getTendonPairParam(PushRecoverService::TendonPairParam& i_param);


    enum PushRecoveryState {PR_IDLE, PR_READY, PR_BUSY, PR_TRANSITION_TO_READY, PR_TRANSITION_TO_IDLE};
    enum PushDetectorState {PD_DISABLE=0, PD_ENABLE=1};
    enum PR_CONTACT_STATE  {BOTH_FOOTS, RFOOT, LFOOT, ON_AIR};
    enum EE_INDEX_LEGS     {EE_INDEX_LEFT=0, EE_INDEX_RIGHT=1};

    /* Assert emergencyStopReqFlag = true from service port */
    bool assertEmergencyStop(void);

    /* change current control state to target_state and enable interpolator */
    bool shiftPushRecoveryState(PushRecoveryState target_state);

    /* Stop thread until the control state to be not busy */
    void waitPushRecoveryStateReady(void);

    /* Start push recovery mode */
    bool startPushRecovery(void);

    /* Stop push recovery mode */
    bool stopPushRecovery(void);

    /* Start push recovery logger */
    bool startLogging(void);

    /* Stop push recovery logger */
    bool stopLogging(void);

    /* Enable/Disable Push Detection */
    bool enablePushDetect(void);
    bool disablePushDetect(void);

    /* PushDetectParam Service function */
    bool setPushDetectParam(const OpenHRP::PushRecoverService::PushDetectParam& i_param);
    bool getPushDetectParam(OpenHRP::PushRecoverService::PushDetectParam& o_param);

    /* OnlineWalkParam Service function */
    bool setOnlineWalkParam(const OpenHRP::PushRecoverService::OnlineWalkParam& i_param);
    bool getOnlineWalkParam(OpenHRP::PushRecoverService::OnlineWalkParam& o_param);

    /* Set wheel control mode */
    bool setWheelMode(const long mode);
    /* WheelControllerParam Service function */
    bool setWheelControllerParam(const OpenHRP::PushRecoverService::WheelControllerParamSet& i_param);
    bool getWheelControllerParam(OpenHRP::PushRecoverService::WheelControllerParamSet& o_param);

    template<class T>
    struct TrajectoryElement {
        T p;
        T body_p;
        T footl_p;
        T footr_p;
        T dp;
        T body_dp;
        T footl_f;
        T footr_f;
        T body_rpy;
        T footl_rpy;
        T footr_rpy;
        TrajectoryElement(){};
        template<class U>
        TrajectoryElement<T>& operator=(const TrajectoryElement<U>& lhs){
            this->p       = lhs.p;
            this->body_p  = lhs.body_p;
            this->footl_p = lhs.footl_p;
            this->footr_p = lhs.footr_p;
            this->footl_f = lhs.footl_f;
            this->footr_f = lhs.footr_f;
            this->dp      = lhs.dp;
            this->body_dp = lhs.body_dp;
            this->body_rpy = lhs.body_rpy;
            this->footl_rpy = lhs.footl_rpy;
            this->footr_rpy = lhs.footr_rpy;
            return *this;
        };
        void clear(void){
            p       = T(0.0,0.0,0.0);
            body_p  = T(0.0,0.0,0.0);
            footl_p = T(0.0,0.0,0.0);
            footr_p = T(0.0,0.0,0.0);
            footl_f = T(0.0,0.0,0.0);
            footr_f = T(0.0,0.0,0.0);
            dp      = T(0.0,0.0,0.0);
            body_dp = T(0.0,0.0,0.0);
            body_rpy = T(0.0,0.0,0.0);
            footl_rpy = T(0.0,0.0,0.0);
            footr_rpy = T(0.0,0.0,0.0);
        };
    };

    struct ModifyTrajectoryContext {
        Vec3 rot;
        Vec3 rot_offset;
        Vec3 filtered_rot;
        Vec3 lpf_rot;
        Vec3 prev_lpf_rot;
        Vec3 lpf_drot;
        Vec3 hpf_rot;
        float transition_gain;
        float transition_gain_diff;
        bool  transition_state;
        enum FZ_TRANSITION_STATE_TYPE{FZ_TRANSITION_IDLE=0, FZ_TRANSITION_TO_READY=1, FZ_TRANSITION_TO_IDLE=2, FZ_TRANSITION_READY=3};
        float fz_transition_gain;
        float fz_transition_gain_diff;
        int  fz_transition_state;
        int fz_contact[2];
        float fz_contact_z[2];
        OpenHRP::PushRecoverService::OnlineWalkParam onlineWalkParam;
        ModifyTrajectoryContext() :
            rot( Vec3Zero() ),
            rot_offset( Vec3Zero() ),
            filtered_rot( Vec3Zero() ),
            lpf_rot( Vec3Zero() ),
            prev_lpf_rot( Vec3Zero() ),
            lpf_drot( Vec3Zero() ),
            hpf_rot( Vec3Zero() ),
            transition_gain( 0.0f ),
            transition_gain_diff( 0.002f ),
            transition_state( true ),
            fz_transition_gain( 0.0f ),
            fz_transition_gain_diff( 0.002f ),
            fz_transition_state( FZ_TRANSITION_IDLE )
        {
            fz_contact[0] = 0;
            fz_contact[1] = 0;
            fz_contact_z[0] = 0.0f;
            fz_contact_z[1] = 0.0f;
            onlineWalkParam.filter_fp = 0.95;
            onlineWalkParam.lpf_fp    = 0.9995;
            onlineWalkParam.modify_rot_gain_x = 0.0;
            onlineWalkParam.modify_rot_gain_y  = 0.0;
            onlineWalkParam.fz_contact_threshold_upper = 15.0;
            onlineWalkParam.fz_contact_threshold_lower = 8.0;
            onlineWalkParam.foot_roll_gain = 0.0;
            onlineWalkParam.foot_pitch_gain = 0.0;
            onlineWalkParam.body_roll_gain = 0.0;
            onlineWalkParam.body_pitch_gain = 0.0;

            onlineWalkParam.owpg_step_time     = 800;
            onlineWalkParam.owpg_step_margin   = 300;
            onlineWalkParam.owpg_max_calclen   = onlineWalkParam.owpg_step_time + 600;
            onlineWalkParam.owpg_iterate_num   = 2;
            onlineWalkParam.owpg_feedback_gain = 0.3;

            onlineWalkParam.owpg_zmp_modify_x_max = 0.55f;
            onlineWalkParam.owpg_zmp_modify_y_max = 0.40f;
            onlineWalkParam.owpg_xstep_max = 0.50f;
            onlineWalkParam.owpg_ystep_max = 0.20f;
            onlineWalkParam.owpg_rot_offset_threshold_x = 3.0f*S_PI/180.0f*0.62f;
            onlineWalkParam.owpg_rot_offset_threshold_y = 3.0f*S_PI/180.0f*0.62f;
            onlineWalkParam.owpg_rot_offset_gain_x = 0.0f;
            onlineWalkParam.owpg_rot_offset_gain_y = 0.5f;
            onlineWalkParam.owpg_x_offset_threshold_x = 20e-3;
            onlineWalkParam.owpg_x_offset_threshold_y = 20e-3;
            onlineWalkParam.owpg_x_offset_gain_x = 0.1f;
            onlineWalkParam.owpg_x_offset_gain_y = 0.1f;
            onlineWalkParam.owpg_Dx_offset_threshold_x = 5e-6;
            onlineWalkParam.owpg_Dx_offset_threshold_y = 5e-6;
            onlineWalkParam.owpg_Dx_offset_gain_x = 0.1f;
            onlineWalkParam.owpg_Dx_offset_gain_y = 0.1f;
            onlineWalkParam.owpg_Dx_offset_acc_thre = 0.01f;
            onlineWalkParam.owpg_modify_first_magnity = 1.0;
            onlineWalkParam.enable = false;
            onlineWalkParam.datal  = 0;
            onlineWalkParam.dataf  = 0.0;
        };
        void reset(){
            rot = Vec3Zero();
            rot_offset = Vec3Zero();
            filtered_rot = Vec3Zero();
            lpf_rot = Vec3Zero();
            prev_lpf_rot = Vec3Zero();
            lpf_drot = Vec3Zero();
            hpf_rot = Vec3Zero();
            transition_gain = 0.0f;
            transition_state = true;
            fz_transition_gain = 0.0f;
            fz_transition_state = FZ_TRANSITION_IDLE;
            fz_contact[0] = 0;
            fz_contact[1] = 0;
            fz_contact_z[0] = 0.0f;
            fz_contact_z[1] = 0.0f;
        };
        void copyWalkParam( OnlinePatternGenerator *p_owpg ) const{
            OnlinePatternGenerator::WalkParam wp;
            wp.step_time     = p_owpg->convMsecToFrame( onlineWalkParam.owpg_step_time );
            wp.step_margin   = p_owpg->convMsecToFrame( onlineWalkParam.owpg_step_margin );
            wp.max_calclen   = onlineWalkParam.owpg_max_calclen;
            wp.iterate_num   = onlineWalkParam.owpg_iterate_num;
            wp.feedback_gain = onlineWalkParam.owpg_feedback_gain;

            wp.zmp_modify_x_max = onlineWalkParam.owpg_zmp_modify_x_max;
            wp.zmp_modify_y_max = onlineWalkParam.owpg_zmp_modify_y_max;
            wp.xstep_max        = onlineWalkParam.owpg_xstep_max;
            wp.ystep_max        = onlineWalkParam.owpg_ystep_max;
            wp.rot_offset_threshold_x = onlineWalkParam.owpg_rot_offset_threshold_x;
            wp.rot_offset_threshold_y = onlineWalkParam.owpg_rot_offset_threshold_y;
            wp.rot_offset_gain_x = onlineWalkParam.owpg_rot_offset_gain_x;
            wp.rot_offset_gain_y = onlineWalkParam.owpg_rot_offset_gain_y;
            wp.x_offset_threshold_x = onlineWalkParam.owpg_x_offset_threshold_x;
            wp.x_offset_threshold_y = onlineWalkParam.owpg_x_offset_threshold_y;
            wp.x_offset_gain_x = onlineWalkParam.owpg_x_offset_gain_x;
            wp.x_offset_gain_y = onlineWalkParam.owpg_x_offset_gain_y;
            wp.Dx_offset_threshold_x = onlineWalkParam.owpg_Dx_offset_threshold_x;
            wp.Dx_offset_threshold_y = onlineWalkParam.owpg_Dx_offset_threshold_y;
            wp.Dx_offset_gain_x = onlineWalkParam.owpg_Dx_offset_gain_x;
            wp.Dx_offset_gain_y = onlineWalkParam.owpg_Dx_offset_gain_y;
            wp.Dx_offset_acc_thre = onlineWalkParam.owpg_Dx_offset_acc_thre;

            p_owpg->setWalkParam(wp);
            //p_owpg->setMaxCalclen( wp.step_time+100 );
            p_owpg->modifyFilterParam( onlineWalkParam.filter_fp,
                                       onlineWalkParam.lpf_fp
                                       );
            p_owpg->modifyFirstMagnity( onlineWalkParam.owpg_modify_first_magnity );
        };
        void copyWalkParam( RobustOnlinePatternGenerator *p_owpg ) const{
            OnlinePatternGenerator::WalkParam wp;
            wp.step_time     = p_owpg->convMsecToFrame( onlineWalkParam.owpg_step_time );
            wp.step_margin   = p_owpg->convMsecToFrame( onlineWalkParam.owpg_step_margin );
            wp.max_calclen   = onlineWalkParam.owpg_max_calclen;
            wp.iterate_num   = onlineWalkParam.owpg_iterate_num;
            wp.feedback_gain = onlineWalkParam.owpg_feedback_gain;

            wp.zmp_modify_x_max = onlineWalkParam.owpg_zmp_modify_x_max;
            wp.zmp_modify_y_max = onlineWalkParam.owpg_zmp_modify_y_max;
            wp.xstep_max        = onlineWalkParam.owpg_xstep_max;
            wp.ystep_max        = onlineWalkParam.owpg_ystep_max;
            wp.rot_offset_threshold_x = onlineWalkParam.owpg_rot_offset_threshold_x;
            wp.rot_offset_threshold_y = onlineWalkParam.owpg_rot_offset_threshold_y;
            wp.rot_offset_gain_x = onlineWalkParam.owpg_rot_offset_gain_x;
            wp.rot_offset_gain_y = onlineWalkParam.owpg_rot_offset_gain_y;
            wp.x_offset_threshold_x = onlineWalkParam.owpg_x_offset_threshold_x;
            wp.x_offset_threshold_y = onlineWalkParam.owpg_x_offset_threshold_y;
            wp.x_offset_gain_x = onlineWalkParam.owpg_x_offset_gain_x;
            wp.x_offset_gain_y = onlineWalkParam.owpg_x_offset_gain_y;
            wp.Dx_offset_threshold_x = onlineWalkParam.owpg_Dx_offset_threshold_x;
            wp.Dx_offset_threshold_y = onlineWalkParam.owpg_Dx_offset_threshold_y;
            wp.Dx_offset_gain_x   = onlineWalkParam.owpg_Dx_offset_gain_x;
            wp.Dx_offset_gain_y   = onlineWalkParam.owpg_Dx_offset_gain_y;
            wp.Dx_offset_acc_thre = onlineWalkParam.owpg_Dx_offset_acc_thre;

            p_owpg->setWalkParam(wp);
            //p_owpg->setMaxCalclen( wp.step_time+100 );
            p_owpg->modifyFilterParam( onlineWalkParam.filter_fp,
                                       onlineWalkParam.lpf_fp
                                       );
            p_owpg->modifyFirstMagnity( onlineWalkParam.owpg_modify_first_magnity );
            p_owpg->setBodyGain(onlineWalkParam.body_roll_gain, onlineWalkParam.body_pitch_gain);
        };
        void copyWalkParam( OpenHRP::PushRecoverService::OnlineWalkParam *o_param ) const{
            o_param->owpg_step_time      = onlineWalkParam.owpg_step_time;
            o_param->owpg_step_margin    = onlineWalkParam.owpg_step_margin;
            o_param->owpg_max_calclen    = onlineWalkParam.owpg_max_calclen;
            o_param->owpg_iterate_num    = onlineWalkParam.owpg_iterate_num;
            o_param->owpg_feedback_gain  = onlineWalkParam.owpg_feedback_gain;

            o_param->filter_fp           = onlineWalkParam.filter_fp;
            o_param->lpf_fp              = onlineWalkParam.lpf_fp;
            o_param->modify_rot_gain_x   = onlineWalkParam.modify_rot_gain_x;
            o_param->modify_rot_gain_y   = onlineWalkParam.modify_rot_gain_y;

            o_param->owpg_zmp_modify_x_max = onlineWalkParam.owpg_zmp_modify_x_max;
            o_param->owpg_zmp_modify_y_max = onlineWalkParam.owpg_zmp_modify_y_max;
            o_param->owpg_xstep_max = onlineWalkParam.owpg_xstep_max;
            o_param->owpg_ystep_max = onlineWalkParam.owpg_ystep_max;
            o_param->owpg_rot_offset_threshold_x = onlineWalkParam.owpg_rot_offset_threshold_x;
            o_param->owpg_rot_offset_threshold_y = onlineWalkParam.owpg_rot_offset_threshold_y;
            o_param->owpg_rot_offset_gain_x    = onlineWalkParam.owpg_rot_offset_gain_x;
            o_param->owpg_rot_offset_gain_y    = onlineWalkParam.owpg_rot_offset_gain_y;
            o_param->owpg_x_offset_threshold_x = onlineWalkParam.owpg_x_offset_threshold_x;
            o_param->owpg_x_offset_threshold_y = onlineWalkParam.owpg_x_offset_threshold_y;
            o_param->owpg_x_offset_gain_x    = onlineWalkParam.owpg_x_offset_gain_x;
            o_param->owpg_x_offset_gain_y    = onlineWalkParam.owpg_x_offset_gain_y;
            o_param->owpg_Dx_offset_threshold_x = onlineWalkParam.owpg_Dx_offset_threshold_x;
            o_param->owpg_Dx_offset_threshold_y = onlineWalkParam.owpg_Dx_offset_threshold_y;
            o_param->owpg_Dx_offset_gain_x      = onlineWalkParam.owpg_Dx_offset_gain_x;
            o_param->owpg_Dx_offset_gain_y      = onlineWalkParam.owpg_Dx_offset_gain_y;
            o_param->owpg_Dx_offset_acc_thre    = onlineWalkParam.owpg_Dx_offset_acc_thre;
            o_param->owpg_modify_first_magnity  = onlineWalkParam.owpg_modify_first_magnity;

            o_param->enable = onlineWalkParam.enable;
            o_param->datal = onlineWalkParam.datal;
            o_param->dataf = onlineWalkParam.dataf;

            o_param->foot_roll_gain             = onlineWalkParam.foot_roll_gain;
            o_param->foot_pitch_gain            = onlineWalkParam.foot_pitch_gain;
            o_param->body_roll_gain             = onlineWalkParam.body_roll_gain;
            o_param->body_pitch_gain            = onlineWalkParam.body_pitch_gain;
            o_param->fz_contact_threshold_upper = onlineWalkParam.fz_contact_threshold_upper;
            o_param->fz_contact_threshold_lower = onlineWalkParam.fz_contact_threshold_lower;
        };
    };

protected:
    // Configuration variable declaration
    // <rtc-template block="config_declare">
    TimedDoubleSeq m_qRef;
    TimedDoubleSeq m_qCurrent;
    std::vector<TimedDoubleSeq> m_force;
    std::vector<TimedDoubleSeq> m_ref_force;
    TimedOrientation3D m_rpy;
    TimedAngularVelocity3D m_rate;
    TimedAcceleration3D m_acc;
    TimedPoint3D m_basePos;
    TimedOrientation3D m_baseRpy;
    TimedPose3D m_basePose;
    TimedPoint3D m_zmp;
    TimedLong m_emergencySignal;

    TimedAcceleration3D m_accRef;
    TimedBooleanSeq m_contactStates;
    TimedDoubleSeq  m_controlSwingSupportTime;
    TimedBoolean    m_walkingStates;
    TimedPoint3D    m_sbpCogOffset;

    TimedFloatSeq   m_joyaxes;
    TimedBooleanSeq m_joybuttons;

    TimedDoubleSeq  m_tauRef;
    TimedDoubleSeq  m_wRef;
    TimedBooleanSeq m_wheel_brake;
    // </rtc-template>

    // DataInPort declaration
    // <rtc-template block="inport_declare">
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
    RTC::InPort<TimedPoint3D> m_basePosIn;
    RTC::InPort<TimedOrientation3D> m_baseRpyIn;
    RTC::InPort<TimedPoint3D> m_zmpIn;
    std::vector<RTC::InPort<TimedDoubleSeq> *> m_forceIn;
    std::vector<RTC::InPort<TimedDoubleSeq> *> m_ref_forceIn;
    RTC::InPort<TimedAngularVelocity3D> m_rateIn;
    RTC::InPort<RTC::TimedAcceleration3D> m_accIn;
    RTC::InPort<TimedOrientation3D> m_rpyIn;
    RTC::InPort<TimedLong> m_emergencySignalIn;

    RTC::InPort<RTC::TimedAcceleration3D> m_accRefIn;
    RTC::InPort<RTC::TimedBooleanSeq> m_contactStatesIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_controlSwingSupportTimeIn;
    RTC::InPort<RTC::TimedBoolean> m_walkingStatesIn;
    RTC::InPort<RTC::TimedPoint3D> m_sbpCogOffsetIn;

    RTC::InPort<RTC::TimedFloatSeq>   m_joyaxesIn;
    RTC::InPort<RTC::TimedBooleanSeq> m_joybuttonsIn;
    // </rtc-template>

    // DataOutPort declaration
    // <rtc-template block="outport_declare">
    // </rtc-template>

    // DataOutPort declaration
    // <rtc-template block="outport_declare">
    RTC::OutPort<RTC::TimedDoubleSeq> m_qRefOut;
    RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosOut;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyOut;
    RTC::OutPort<RTC::TimedPose3D> m_basePoseOut;

    std::vector<RTC::OutPort<TimedDoubleSeq> *> m_ref_forceOut;
    RTC::OutPort<RTC::TimedAcceleration3D> m_accRefOut;
    RTC::OutPort<RTC::TimedBooleanSeq> m_contactStatesOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_controlSwingSupportTimeOut;
    RTC::OutPort<RTC::TimedBoolean> m_walkingStatesOut;
    RTC::OutPort<RTC::TimedPoint3D> m_sbpCogOffsetOut;

    RTC::OutPort<RTC::TimedDoubleSeq> m_tauRefOut;

    RTC::OutPort<RTC::TimedDoubleSeq> m_wRefOut;
    RTC::OutPort<RTC::TimedBooleanSeq> m_wheel_brakeOut;
    // </rtc-template>

    // CORBA Port declaration
    // <rtc-template block="corbaport_declare">

    // </rtc-template>

    // Service declaration
    // <rtc-template block="service_declare">
    RTC::CorbaPort m_PushRecoverServicePort;

    // </rtc-template>

    // Consumer declaration
    // <rtc-template block="consumer_declare">
    PushRecoverService_impl m_service0;

    // </rtc-template>

private:
    const float m_dt_gen;
    double m_dt;
    double m_dt_i;
    double m_mg, m_mg2; /* m_robot->totalMass() */
    coil::Mutex m_mutex;
    std::map<std::string, hrp::Vector3> abs_forces, abs_moments, abs_ref_forces, abs_ref_moments;
    hrp::BodyPtr m_robot, m_robot_current;
    unsigned int m_debugLevel;
    unsigned int m_expectedJointNum;
    unsigned int m_simmode;
    unsigned int m_generator_select;
    bool emergencyStopReqFlag;
    int loop;

    hrp::JointPathExPtr m_pleg[2];

    /* PushDetectParam */
    PushDetectorState pushDetector_state;
    OpenHRP::PushRecoverService::PushDetectParam pushDetectParam;

    /* Input data buffer */
    hrp::Vector3  input_zmp, input_basePos;
    hrp::Matrix33 input_baseRot;

    /* Actual Data calced from input data */
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_cogvel_filter;
    hrp::Vector3  m_act_zmp,       prev_act_zmp;
    hrp::Vector3  m_rel_act_zmp;
    hrp::Vector3  m_act_root_pos,    m_prev_act_root_pos,   m_prev_act_root_pos_base;
    hrp::Vector3  m_act_rootvel, m_prev_act_rootvel;
    hrp::Vector3  m_act_world_root_pos;
    hrp::Vector3  m_act_cog,         m_prev_act_cog,    m_prev_act_cog_base;
    hrp::Vector3  m_act_cogvel,      m_prev_act_cogvel;
    hrp::Vector3  act_base_rpy;
    hrp::Matrix33 m_prev_act_foot_origin_rot;
    double        m_prev_act_force_z[2]; /* previous force z used in calcZMP */

    /* Estimated state data */
    hrp::Vector3 m_est_cogvel_from_rpy;

    JointAngleBuffer<LegIKParam> m_ready_joint_angle;

    /* Joystick WatchDog Timer */
    struct JoyState {
        bool enabled;
        bool keep_idle;
        int  wdt;
        JoyCommand prev_command;
        JoyState() :
            enabled(false), wdt(0), keep_idle(false) {};
        void reset(void) {
            enabled = false;
            keep_idle = false;
            wdt = 0;
            prev_command.x = 0.0;
            prev_command.y = 0.0;
        };
    };
    JoyState m_joystate;

    /* Reference Data buffer */
    //double        *ref_q,   *prev_ref_q;
    //double        m_ref_q[12],   m_prev_ref_q[12], m_ref_q_store[12];
    JointAngleBuffer<LegIKParam> m_ref_q, m_prev_ref_q, m_ref_q_store;
    hrp::Vector3  m_rel_ref_zmp, m_prev_rel_ref_zmp; // ref zmp in base frame
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > ref_zmp_modif_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > ref_basePos_modif_filter;
    hrp::Vector3  m_ref_zmp, ref_basePos, prev_ref_zmp, m_prev_ref_basePos, ref_zmp_modif, m_ref_basePos_modif;
    hrp::Matrix33     ref_baseRot;
    hrp::Vector3      prev_imu_sensor_pos, prev_imu_sensor_vel;
    hrp::Vector3      m_ref_cog, m_prev_ref_cog, m_ref_cogvel;
    hrp::Vector3      m_ref_force_vec[2];
    TrajectoryElement<hrp::Vector3e>  m_prev_ref_traj;

    interpolator *transition_interpolator;
    double transition_interpolator_ratio;
    PushRecoveryState m_current_control_state;

    /* indexed robot model parts names */
    struct EElinkParam {
        std::string   fsensor_name;
        std::string   ee_name;
        std::string   ee_target;
        std::string   ee_base;
        hrp::Vector3  ee_localp;
        hrp::Matrix33 ee_localR;
        bool          act_contact_state;
    };
    PR_CONTACT_STATE m_foots_contact_states, m_prev_foots_contact_states;
    std::map<std::string, size_t> ee_index_map;
    int ee_index_lr[2];
    std::vector<EElinkParam> ee_params;
    hrp::Vector3 world_sensor_ps[2];
    hrp::Vector3 world_force_ps[2];
    hrp::Vector3 world_force_ms[2];

    IIKMethod* m_pIKMethod;
    hrp::Vector3 m_body_p_at_start, m_body_p_diff_at_start, m_footl_p_at_start, m_footr_p_at_start;
    hrp::Vector3 m_basePos_modif_at_start;
    FrameRateMatcher rate_matcher;

    ReactivePatternGenerator stpf;
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
    OnlinePatternGenerator   m_owpg;
#else
    RobustOnlinePatternGenerator   m_owpg;
#endif
#if defined(USE_APECF)
    AbsolutePoseEstimator_CF<LegIKParam,LEG_IK_TYPE>    m_abs_est;
#else
    AbsolutePoseEstimator<LegIKParam,LEG_IK_TYPE>    m_abs_est;
#endif
    //boost::array<float, 12> qs, Dqs, DDqs;

    WheelLeg::WheelControllerPtr m_wheel_ctrl;

    BodyControlContext ctx, cty;
    PatternState m_owpg_state;
    ControlState m_owpg_cstate;
    bool m_prev_owpg_isComplete;
    ModifyTrajectoryContext m_modify_rot_context;

    struct ControlBodyComplianceContext {
        double prev_u;
    };
    ControlBodyComplianceContext bodyComplianceContext[2];

    void updateInputData(const bool shw_msg_flag = false);
    hrp::Vector3 updateEstimatedInputData(void);
    void updateEstimatedOutputData(void);
    void setTargetDataWithInterpolation(void);
    void setOutputData(const bool shw_msg_flag = false);

    bool calcWorldForceVector(void);
    PR_CONTACT_STATE calcFootOriginCoords (hrp::Vector3& foot_origin_pos,
                                           hrp::Matrix33& foot_origin_rot) const;
    bool calcZMP(hrp::Vector3& ret_zmp, const double zmp_z) const;
    hrp::Vector3 calcRelActZMP(const hrp::Vector3& foot_zmp, const hrp::Vector3& root_pos, const hrp::Matrix33 root_R) const{
        // convert absolute (in st) -> root-link relative
        return root_R.transpose() * (foot_zmp - root_pos);
    };
#if 0
    hrp::Vector3 calcActCoG_CoGVel(const hrp::Vector3 act_foot_origin_pos,
                                   const hrp::Matrix33 act_foot_origin_rot,
                                   hrp::Vector3& act_cog) const;
#else
    hrp::Vector3 calcActCoG_CoGVel(const hrp::Vector3 act_foot_origin_pos,
                                   const hrp::Matrix33 act_foot_origin_rot,
                                   hrp::Vector3& act_cog,
                                   hrp::Vector3& prev_act_cog_base) const;
#endif
#if 0
    hrp::Vector3 calcActRootPos(const hrp::Vector3 act_foot_origin_pos, const hrp::Matrix33 act_foot_origin_rot) const {
        return act_foot_origin_rot.transpose() * (-act_foot_origin_pos);
    };
#else
    hrp::Vector3 calcActRootPos(const hrp::Vector3 act_foot_origin_pos, const hrp::Matrix33 act_foot_origin_rot, hrp::Vector3& act_root_pos, hrp::Vector3& prev_act_root_base) const;
#endif
    bool updateToCurrentRobotPose(void);
    bool checkJointVelocity(void);
    bool checkBodyPosMergin(const double threshold2, const int loop, const bool mask = false) const;
    bool controlBodyCompliance(bool is_enable);
    void trajectoryReset(void);
    /* ============================================== */
    /* checkEmergencyFlag()                           */
    /* out:                                           */
    /*    :bool                                       */
    /*     EmergencyStopRequestが来ていたらtrueを返す  */
    /*     EmergencyStopReqFlagはfalseになおす。       */
    /* ============================================== */
    bool checkEmergencyFlag(void);
#define USE_DATALOG
    //boost::shared_ptr<SimpleLogger> slogger;
#ifdef USE_DATALOG
    //SimpleLogger *slogger;
    //SimpleLogger::DataLog  dlog;
    data_logger_online<dlog::DataLog_20171023,true> dlogger;
    //data_logger_online<dlog::DataLog,true> dlogger;
    //dlog::DataLog dlog;
    dlog::DataLog_20171023 dlog;
#endif
    bool                   dlog_save_flag;
    struct timeval         stv; /* time of OnInitialized */

    inline void executeActiveStateExtractTrajectory(const bool suppress_pr,
                                                    const bool on_ground,
                                                    const hrp::Vector3 &ref_basePos_modif,
                                                    TrajectoryElement<Vec3e> &ref_traj);
    inline void executeActiveStateExtractTrajectoryOnline(const bool on_ground,
                                                          const hrp::Vector3 &ref_basePos_modif,
                                                          TrajectoryElement<Vec3e> &ref_traj);
    inline void modifyTrajectoryRot(const bool enable_modify, const bool on_ground,
                                    ModifyTrajectoryContext &context,
                                    TrajectoryElement<Vec3e> &ref_traj) const;
    inline void updateRefPos_ZMP(const TrajectoryElement<Vec3e> &ref_traj,
                                 const hrp::Vector3 &ref_basePos_modif,
                                 hrp::Vector3 &act_world_root_pos);
    inline void updateRotContext( const Vec3 rpy,
                                  ModifyTrajectoryContext &context ) const;
    inline void executeActiveStateCalcJointAngle(const TrajectoryElement<Vec3e> &ref_traj,
                                                 const hrp::Vector3 &ref_basePos_modif);
    inline void selectSimmodeControlValue(const int mode, hrp::Vector3 &basepos_modif, hrp::Vector3 &basepos_vel);
    inline void extractSwingSupportTime(double* time){
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
        const F64vec2 tv = m_owpg.getSwingSupportTime(0);
#else
        const F64vec2 tv = m_owpg.getSwingSupportTime(0);
#endif
        time[0] = tv[0];
        time[1] = tv[1];
    };
    inline void modifyFootHeight(const bool on_ground, ModifyTrajectoryContext &context, TrajectoryElement<Vec3e> &ref_traj);
    inline void interpretJoystickCommandandSend(const TimedFloatSeq &axes, const TimedBooleanSeq &buttons, JoyState &jstate, OnlinePatternGenerator* p_owpg) const;
    inline void interpretJoystickCommandandSend(const TimedFloatSeq &axes, const TimedBooleanSeq &buttons, JoyState &jstate, RobustOnlinePatternGenerator* p_owpg) const;
    void printMembers(const int cycle);
};


void PushRecover::executeActiveStateExtractTrajectory(const bool suppress_pr, const bool on_ground, const hrp::Vector3 &ref_basePos_modif, TrajectoryElement<Vec3e> &ref_traj){
    const double threshold  = 70;
    const double threshold2 = (threshold*threshold);

    /* check the state */
    const bool checkBodyPosflag = checkBodyPosMergin(threshold2, loop, on_ground & (pushDetector_state==PD_ENABLE));
    /* Check if Emergency Stop Flag is active */
    const bool emergency_flag = checkEmergencyFlag();

    const bool invoke_pr = (emergency_flag || checkBodyPosflag) && !suppress_pr;

    const float diff_x = m_act_root_pos(0) - m_prev_ref_basePos(0);
    const float diff_y = m_act_root_pos(1) - m_prev_ref_basePos(1);
    const float diff_z = m_act_root_pos(2) - Zc; /* TODO 効果の検証 */

    /* x0[3] = [ p0, x0, Dx0] */
    const Vec3 x0[] = {
        Vec3(0.0f, 0.0, 0.0f),
        Vec3( pushDetectParam.x_gain_0 * ref_basePos_modif(0), pushDetectParam.x_gain_1 * ref_basePos_modif(1), 0.0f ),
        Vec3( (float)(pushDetectParam.dx_gain_0 * m_act_cogvel(0)), -(float)(pushDetectParam.dx_gain_1 * m_act_cogvel(1)), 0.0f)
    };


    /* TODO set rootLink position default value */
    m_robot->rootLink()->p = hrp::Vector3(traj_body_init[0], traj_body_init[1], traj_body_init[2]+LegIKParam::InitialLfoot_p[2]) + ref_basePos_modif;
    /* save current reference rootlink position for checkBodyPosMergin */
    m_prev_ref_basePos = m_robot->rootLink()->p;

    //m_robot->rootLink()->p = m_act_root_pos;にはならない．ここは理想的なposを入れる
    /* set body_p to m_robot loot link Rotation */
    //m_robot->rootLink()->R = input_baseRot; /* TODO */
    m_robot->rootLink()->R = hrp::Matrix33::Identity(); /* TODO */

    if(invoke_pr){
        std::cout << "[" << m_profile.instance_name << "] " << MAKE_CHAR_COLOR_RED << "Calling ReactivePatternGenerator start" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
        /* Save Current base position */
        m_body_p_at_start = m_act_world_root_pos;
        m_body_p_diff_at_start = hrp::Vector3( diff_x, diff_y, diff_z );
        m_basePos_modif_at_start = hrp::Vector3(pushDetectParam.x_gain_0 * ref_basePos_modif(0),
                                                pushDetectParam.x_gain_1 * ref_basePos_modif(1),
                                                ref_basePos_modif(2)
                                                );
        stpf.start(x0);
        rate_matcher.setCurrentFrame(0);
        m_current_control_state = PR_BUSY;
    }

    /* Get reference trajectory */
    {
        Vec3 sf_pref;
        Vec3 sf_body_p;
        Vec3 sf_footl_p;
        Vec3 sf_footr_p;
        ITrajectoryGenerator* gen = stpf.getReady();
        if(m_current_control_state == PR_BUSY && gen!=0){
            rate_matcher.incrementFrame();
            gen->getTrajectoryFrame(rate_matcher.getConvertedFrame(),
                                    sf_pref,
                                    sf_body_p,
                                    sf_footl_p,
                                    sf_footr_p );
            ref_traj.p       = sf_pref;
            ref_traj.body_p  = sf_body_p;
            ref_traj.footl_p = sf_footl_p;
            ref_traj.footr_p = sf_footr_p;
        }else{
            ref_traj = m_prev_ref_traj;  /* keep current trajectory state */
        }
    }

    /* calc the trajectory velocity dp and body_dp */
    {
        ref_traj.dp      = (ref_traj.p      - ((Vec3e)m_prev_ref_traj.p))      * m_dt_i;
        ref_traj.body_dp = (ref_traj.body_p - ((Vec3e)m_prev_ref_traj.body_p)) * m_dt_i;
    }
    m_prev_ref_traj = ref_traj;

}; /* executeActiveStateExtractTrajectory */

void PushRecover::executeActiveStateExtractTrajectoryOnline(const bool on_ground,
                                                            const hrp::Vector3 &ref_basePos_modif,
                                                            TrajectoryElement<Vec3e> &ref_traj){
    // /* Check if Emergency Stop Flag is active */
    const bool emergency_flag = checkEmergencyFlag();
    const bool invoke_pr = emergency_flag;

    /* TODO set rootLink position default value */
    //m_robot->rootLink()->p = hrp::Vector3(traj_body_init[0], traj_body_init[1], traj_body_init[2]+LegIKParam::InitialLfoot_p[2]) + ref_basePos_modif;
    m_robot->rootLink()->p = hrp::Vector3(traj_body_init[0], traj_body_init[1], traj_body_init[2]+LegIKParam::InitialLfoot_p[2]);
    /* save current reference rootlink position for checkBodyPosMergin */
    m_prev_ref_basePos = m_robot->rootLink()->p;

    //m_robot->rootLink()->p = m_act_root_pos;にはならない．ここは理想的なposを入れる
    /* set body_p to m_robot loot link Rotation */
    //m_robot->rootLink()->R = input_baseRot; /* TODO */
    m_robot->rootLink()->R = hrp::Matrix33::Identity(); /* TODO */

    if(invoke_pr){
        std::cout << "Pushed Steps" << std::endl;
        {
            const float ystep = LegIKParam::InitialLfoot_p[1];
            const int   step_time = m_modify_rot_context.onlineWalkParam.owpg_step_time;
            StepCommandList steps;
            switch(m_simmode){
            case 0:
                std::cout << "[pr] StopCommand" << std::endl;
                break;
            case 1:
                std::cout << "[pr] DEMO Step" << std::endl;
                std::cout << "[pr] Forward Step(Right first)" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.005f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.005f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(-0.005f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(-0.005f, +ystep*2 , 0.0f)) );
                break;
            case 2:
                std::cout << "[pr] Forward Step(Right first)" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.08f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.08f, +ystep*2 , 0.0f)) );
                break;
            case 3:
                std::cout << "[pr] Back Step(Right first)" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(-0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(-0.08f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(-0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(-0.08f, +ystep*2 , 0.0f)) );
                break;
            case 4:
                std::cout << "[pr] Forward Step(Left first)" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.08f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.08f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.08f, -ystep*2 , 0.0f)) );
                break;
            case 5:
                std::cout << "[pr] Back Step(Left first)" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(-0.08f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(-0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(-0.08f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(-0.08f, -ystep*2 , 0.0f)) );
                break;
            case 6:
                std::cout << "[pr] Left Step" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.0f, 0.06+ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.0f, 0.06-ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.0f, 0.06+ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.0f, 0.06-ystep*2 , 0.0f)) );
                break;
            case 7:
                std::cout << "[pr] Right Step" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.0f, -0.06-ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.0f, -0.06+ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.0f, -0.06-ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.0f, -0.06+ystep*2 , 0.0f)) );
                break;
            case 8:
                std::cout << "[pr] Fast Forward Step(Right first)" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(500), -1, Vec3(0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(500),  1, Vec3(0.08f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(500), -1, Vec3(0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(500),  1, Vec3(0.08f, +ystep*2 , 0.0f)) );
                break;
            case 9:
                std::cout << "[pr] switched to idle mode" << std::endl;
                break;
            default:
                std::cout << "[pr] default step" << std::endl;
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.08f, +ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time), -1, Vec3(0.08f, -ystep*2 , 0.0f)) );
                steps.push_back( (UnitStepCommandPtr) new UnitStepCommand(m_owpg.convMsecToFrame(step_time),  1, Vec3(0.08f, +ystep*2 , 0.0f)) );
            }
            if(steps.size()>0){
                m_owpg.pushSteps( steps );
            }
        }
    }

    if(m_owpg.isComplete() == true && m_prev_owpg_isComplete == false){
        const Vec3 body_p_diff_offset = m_owpg_state.body_p - (Vec3)((m_owpg_state.foot_l_p+m_owpg_state.foot_r_p).array()*0.5f);
        m_body_p_at_start      += hrp::Vector3(m_owpg_state.body_p[0],
                                               m_owpg_state.body_p[1],
                                               m_owpg_state.body_p[2])
            -
            hrp::Vector3(body_p_diff_offset[0],
                         body_p_diff_offset[1],
                         body_p_diff_offset[2]);
        m_footl_p_at_start     += hrp::Vector3(m_owpg_state.foot_l_p_mod[0],
                                               m_owpg_state.foot_l_p_mod[1],
                                               m_owpg_state.foot_l_p_mod[2]);
        m_footr_p_at_start     += hrp::Vector3(m_owpg_state.foot_r_p_mod[0],
                                               m_owpg_state.foot_r_p_mod[1],
                                               m_owpg_state.foot_r_p_mod[2]);
        rate_matcher.setCurrentFrame(0);
        //m_abs_est.reset_estimation<false>(&m_ready_joint_angle[0]);
    }
    m_prev_owpg_isComplete = m_owpg.isComplete();


#if defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
    {
        PoseState pose_state;
        m_abs_est.getAbsolutePoseState(pose_state);

        const Vec3 body_p_offset = Vec3::Zero() + Vec3(traj_body_init[0], 0.0f, 0.0f);
        const Vec3 footl_p_offset = Vec3::Zero();
        const Vec3 footr_p_offset = Vec3::Zero();
        UpdateState ustate;
        ustate.pref     = Vec3::Zero();
        ustate.zmp      = pose_state.zmp;
        ustate.com_p    = -body_p_offset + Vec3(pose_state.body.p[0], pose_state.body.p[1], 0.0f);
        ustate.com_dp   = pose_state.body.Dp;
#if 0
        ustate.rot      =  bodylink::rot2rpy<bodylink::ROBOTICS>(pose_state.body.R);
#else
        /* TODO:: Setting usate.rot is not looks like wrong, but there are some mistakes in using body R. */
        //ustate.rot      =  pose_state.body.rp_filtered;
        ustate.rot = Vec3::Zero();
#endif
        ustate.rate     = pose_state.body.rate;
        ustate.foot_l_p = -footl_p_offset + pose_state.footl.p;
        ustate.foot_r_p = -footr_p_offset + pose_state.footr.p;
        ustate.foot_contact_state[0] = pose_state.foot_state[0];
        ustate.foot_contact_state[1] = pose_state.foot_state[1];
        ustate.foot_l_rpy = bodylink::rot2rpy<bodylink::ROBOTICS>(pose_state.footl.R);
        ustate.foot_r_rpy = bodylink::rot2rpy<bodylink::ROBOTICS>(pose_state.footr.R);
        m_owpg.update(ustate);
    }
#endif

    //const int inc_frame = (int)(m_dt/m_dt_gen);
    const int inc_frame = 1;
    if(loop%500==0){
        std::cout << "inc_frame = " << inc_frame << std::endl;
        std::cout << "m_dt = " << m_dt << std::endl;
        std::cout << "m_dt_gen = " << m_dt_gen << std::endl;
    }
    {
        //const bool idle_state = m_simmode==0?false:true; /* TODO */
        const bool idle_state = m_joystate.keep_idle?true:m_simmode==0?false:true; /* TODO */
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
        m_owpg.incrementFrameNoIdle2<LegIKParam,LEG_IK_TYPE,0>(inc_frame, m_owpg_state, m_owpg_cstate, idle_state);
#else
        const bool use_iterate = true;
        //const bool use_iterate = false;
        m_owpg.incrementFrame<LegIKParam, LEG_IK_TYPE, use_iterate, 0>(inc_frame, m_owpg_state, idle_state);
#endif
    }

    ref_traj.p       = Vec3(m_owpg_state.p);
    ref_traj.body_p  = Vec3(m_owpg_state.body_p);
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
    ref_traj.footl_p = Vec3(m_owpg_state.foot_l_p);
    ref_traj.footr_p = Vec3(m_owpg_state.foot_r_p);
#else
    ref_traj.footl_p = Vec3(m_owpg_state.foot_l_p_mod);
    ref_traj.footr_p = Vec3(m_owpg_state.foot_r_p_mod);
#endif

    ref_traj.footl_f = Vec3(m_owpg_state.foot_l_f);
    ref_traj.footr_f = Vec3(m_owpg_state.foot_r_f);

#if defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
    ref_traj.body_rpy  = Vec3(m_owpg_state.body_rpy);
    ref_traj.footl_rpy = m_owpg_state.foot_l_rpy_mod;
    ref_traj.footr_rpy = m_owpg_state.foot_r_rpy_mod;
    if(loop%500==0){
        printf("m_owpg_state.body_rpy[0,1] = [%5.4f, %5.4f]\n", m_owpg_state.body_rpy[0], m_owpg_state.body_rpy[1]);
        printf("ref_traj.body_rpy[0,1]     = [%5.4f, %5.4f]\n", ref_traj.body_rpy[0], ref_traj.body_rpy[1]);
    }
#endif

#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
    {
        const Vec3 rot_rp  = Vec3(act_base_rpy[0], act_base_rpy[1], act_base_rpy[2] );
        const Vec3 rate    = Vec3(0.0f,0.0f,0.0f);
        const Vec3 zmp     = Vec3(m_ref_zmp[0], m_ref_zmp[1], m_ref_zmp[2]);
        UpdateState ustate = { Vec3(m_prev_ref_traj.p[0],m_prev_ref_traj.p[1],m_prev_ref_traj.p[2]),
                               zmp,
                               Vec3(m_prev_ref_traj.body_p[0],m_prev_ref_traj.body_p[1],m_prev_ref_traj.body_p[2]),
                               Vec3::Zero(),
                               rot_rp,
                               rate };
        m_owpg.update(ustate);
        m_owpg_cstate.pxstate->body_q[0] = m_owpg_state.body_p[0];
        m_owpg_cstate.pystate->body_q[0] = m_owpg_state.body_p[1];
        //m_owpg_cstate.pxstate->body_q[1] = m_owpg_state.body_p[0] - body_p_prev[0];
        //m_owpg_cstate.pystate->body_q[1] = m_owpg_state.body_p[1] - body_p_prev[1];
        /* TODO */
        m_owpg_cstate.pxstate->body_q[1] = m_act_cogvel[0];
        m_owpg_cstate.pystate->body_q[1] = m_act_cogvel[1];
    }
#endif

    /* calc the trajectory velocity dp and body_dp */
    {
        ref_traj.dp      = (ref_traj.p      - ((Vec3e)m_prev_ref_traj.p))      * m_dt_i;
        ref_traj.body_dp = (ref_traj.body_p - ((Vec3e)m_prev_ref_traj.body_p)) * m_dt_i;
    }
    m_prev_ref_traj = ref_traj;

}; /* executeActiveStateExtractTrajectorynline */


void PushRecover::executeActiveStateCalcJointAngle(const TrajectoryElement<Vec3e> &ref_traj,
                                                   const hrp::Vector3 &ref_basePos_modif){
    // 新しい target_joint_angleを計算
    _MM_ALIGN16 float target_joint_angle[12];
#if defined(__INTEL_COMPILER)||defined(__ICC)
    _MM_ALIGN16 Mat3 body_R            = Mat3::identity();
#elif defined(__GNUC__)
    //TODO ここはIdentityを使うよりもKFからの姿勢を使うのが良いかも。
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
    _MM_ALIGN16 float c[4],s[4];
    bodylink::sincos_ps( bodylink::F32vec4(m_modify_rot_context.rot_offset[0] * m_modify_rot_context.onlineWalkParam.body_roll_gain,
                                           m_modify_rot_context.rot_offset[1] * m_modify_rot_context.onlineWalkParam.body_pitch_gain,
                                           m_modify_rot_context.rot_offset[2],
                                           0.0f),
                         (bodylink::v4sf*)s, (bodylink::v4sf*)c );
    const Mat3 body_R                  = rotateMat3<1>( c[1], s[1] )*rotateMat3<0>( c[0], s[0] );
#ifdef DEBUG_HOGE
    if(loop%500==0){
        printf("c[0,1]=[%5.4f, %5.4f]\n",c[0],c[1]);
        printf("s[0,1]=[%5.4f, %5.4f]\n",s[0],s[1]);
        std::cout << "body_R=\n" << body_R << std::endl;
    }
#endif
#elif 1
    _MM_ALIGN16 float c[4],s[4];
    bodylink::sincos_ps( bodylink::F32vec4(ref_traj.body_rpy[0] * m_modify_rot_context.onlineWalkParam.body_roll_gain,
                                           ref_traj.body_rpy[1] * m_modify_rot_context.onlineWalkParam.body_pitch_gain,
                                           m_modify_rot_context.rot_offset[2],
                                           0.0f),
                         (bodylink::v4sf*)s, (bodylink::v4sf*)c );
    const Mat3 body_R                  = rotateMat3<1>( c[1], s[1] )*rotateMat3<0>( c[0], s[0] );
#ifdef DEBUG_HOGE
    if(loop%500==0){
        printf("ref_traj.body_rpy[0,1] = [%5.4f, %5.4f]\n", ref_traj.body_rpy[0], ref_traj.body_rpy[1]);
        printf("body_roll_gain = %5.4f, body_pitch_gain=%5.4f\n", m_modify_rot_context.onlineWalkParam.body_roll_gain, m_modify_rot_context.onlineWalkParam.body_pitch_gain);
        printf("c[0,1]=[%5.4f, %5.4f]\n",c[0],c[1]);
        printf("s[0,1]=[%5.4f, %5.4f]\n",s[0],s[1]);
        std::cout << "body_R=\n" << body_R << std::endl;
    }
#endif
#elif 0
    _MM_ALIGN16 float c[4],s[4];
    bodylink::sincos_ps( bodylink::F32vec4(m_modify_rot_context.lpf_rot[0]*-0.900f,
                                           m_modify_rot_context.lpf_rot[1]*-0.900f,
                                           m_modify_rot_context.lpf_rot[2],
                                           0.0f),
                         (bodylink::v4sf*)s, (bodylink::v4sf*)c );

    const Mat3 body_R                  = rotateMat3<1>( c[1], s[1] )*rotateMat3<0>( c[0], s[0] );
#else
    const Mat3 body_R                  = Mat3::Identity();
#endif
#endif
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
    float foot_l_roll;
    float foot_r_roll;
    if(m_modify_rot_context.transition_state){
        foot_l_roll            = -m_modify_rot_context.filtered_rot[0] * m_modify_rot_context.transition_gain * m_modify_rot_context.onlineWalkParam.foot_roll_gain;
        foot_r_roll            = -m_modify_rot_context.filtered_rot[0] * m_modify_rot_context.transition_gain * m_modify_rot_context.onlineWalkParam.foot_roll_gain;
    }else{
        foot_l_roll            = -m_modify_rot_context.filtered_rot[0] * m_modify_rot_context.onlineWalkParam.foot_roll_gain;
        foot_r_roll            = -m_modify_rot_context.filtered_rot[0] * m_modify_rot_context.onlineWalkParam.foot_roll_gain;
    }
    float foot_l_pitch;
    float foot_r_pitch;
    if(m_modify_rot_context.transition_state){
        foot_l_pitch            = -m_modify_rot_context.filtered_rot[1] * m_modify_rot_context.transition_gain * m_modify_rot_context.onlineWalkParam.foot_pitch_gain;
        foot_r_pitch            = -m_modify_rot_context.filtered_rot[1] * m_modify_rot_context.transition_gain * m_modify_rot_context.onlineWalkParam.foot_pitch_gain;
    }else{
        foot_l_pitch            = -m_modify_rot_context.filtered_rot[1] * m_modify_rot_context.onlineWalkParam.foot_pitch_gain;
        foot_r_pitch            = -m_modify_rot_context.filtered_rot[1] * m_modify_rot_context.onlineWalkParam.foot_pitch_gain;
    }
#elif 1
    float foot_l_roll, foot_r_roll;
    float foot_l_pitch, foot_r_pitch;
    foot_l_roll = ref_traj.footl_rpy[0];
    foot_r_roll = ref_traj.footr_rpy[0];
    foot_l_pitch = ref_traj.footl_rpy[1];
    foot_r_pitch = ref_traj.footr_rpy[1];
#elif 0
    float foot_l_roll, foot_r_roll;
    float foot_l_pitch, foot_r_pitch;
    //foot_l_roll = foot_r_roll = m_modify_rot_context.rot_offset[0];
    //foot_l_pitch = foot_r_pitch = m_modify_rot_context.rot_offset[1];
    foot_l_roll = foot_r_roll = 0.0f;
    foot_l_pitch = foot_r_pitch = 0.0f;
#else
#error "foot rpy is not calculated"
#endif

    //foot_l_pitch += LegIKParam::InitialLfoot_pitch;
    //foot_r_pitch += LegIKParam::InitialRfoot_pitch;

    m_pIKMethod->calcik_ini(body_R,
                            body_p_default_offset + ref_traj.body_p,
                            ref_traj.footl_p,
                            LegIKParam::InitialLfoot_p,
                            ref_traj.footr_p,
                            LegIKParam::InitialRfoot_p,
                            LegIKParam::InitialLfoot_R,
                            LegIKParam::InitialRfoot_R,
                            foot_l_pitch,
                            foot_r_pitch,
                            foot_l_roll,
                            foot_r_roll,
                            target_joint_angle );

#if ROBOT==0
    //bool error_flag = false;
    /* set target_joint_angle */
    for(int i=0;i < 12; i++){
        m_robot->joint(i)->q = target_joint_angle[i];  /* rad to rad */
        //g_ready_joint_angle[i] = target_joint_angle[i];
    }
#elif ROBOT==1
    //bool error_flag = false;
    /* set target_joint_angle */
    for(int i=0;i < 6; i++){
        /* m_robot of L1 starts from right leg. */
        m_robot->joint(i)->q   = target_joint_angle[i+6];  /* rad to rad */
        m_robot->joint(i+6)->q = target_joint_angle[i];  /* rad to rad */
        /* ready_joint_angle is the same alignment. */
        // g_ready_joint_angle[i]   = target_joint_angle[i];
        // g_ready_joint_angle[i+6] = target_joint_angle[i+6];
    }
#elif ROBOT==2
    //bool error_flag = false;
    /* set target_joint_angle */
    for(int i=0;i < 6; i++){
        /* m_robot of L1W starts from right leg. */
        m_robot->joint(i)->q   = target_joint_angle[i+6];  /* rad to rad */
        m_robot->joint(i+6)->q = target_joint_angle[i];  /* rad to rad */
    }
    //m_robot->joint(12)->q = m_robot->joint(13)->q = (loop%2000<1000)?bodylink::S_PI/10.0f:-bodylink::S_PI/10.0f;
#else
#error "PushRecover setting m_robot->joint(i)-q. Undefined ROBOT Type"
#endif
}; /* executeActiveStateCalcJointAngle */

void PushRecover::updateRefPos_ZMP(const TrajectoryElement<Vec3e> &ref_traj,
                                   const hrp::Vector3 &ref_basePos_modif,
                                   hrp::Vector3 &act_world_root_pos){
    /* todo : check the continuity of act_world_root_pos */
    /* calc body_p on world coords */
    act_world_root_pos = hrp::Vector3(ref_traj.body_p[0] + m_body_p_at_start(0),
                                      ref_traj.body_p[1] + m_body_p_at_start(1),
                                      ref_traj.body_p[2] + m_body_p_at_start(2));
    /* calc body_p on base frame coords */
    hrp::Vector3 body_p = hrp::Vector3(traj_body_init[0] + ref_traj.body_p[0],
                                       traj_body_init[1] + ref_traj.body_p[1],
                                       traj_body_init[2] + LegIKParam::InitialLfoot_p[2] + ref_traj.body_p[2]);
    //body_p += ref_basePos_modif;

    /* set body_p to m_robot loot link Rotation */
    //m_robot->rootLink()->R = input_baseRot; /* TODO */
    const hrp::Matrix33 body_R = hrp::Matrix33::Identity();

    /* calc Reference ZMP relative to base_frame(Loot link)  */
    const hrp::Vector3 default_zmp_offset(default_zmp_offset_l[0],default_zmp_offset_l[1],default_zmp_offset_l[2]);
#if 1
    m_ref_zmp     = hrp::Vector3(ref_traj.p[0],ref_traj.p[1],ref_traj.p[2]) + ref_zmp_modif + default_zmp_offset;
#else
    m_ref_zmp     = hrp::Vector3(ref_traj.p[0],ref_traj.p[1],ref_traj.p[2]);
#endif
    m_rel_ref_zmp = (body_R.transpose() * (m_ref_zmp - body_p)) - default_zmp_offset;
}; /* updateRefPos_ZMP */

void PushRecover::updateRotContext( const Vec3 rpy,
                                    ModifyTrajectoryContext &context ) const{
    context.rot = rpy;
    m_owpg.signal_filter( context.filtered_rot, rpy ); /* 体幹姿勢の更新 */
    m_owpg.lpf_filter( context.lpf_rot, rpy );     /* 体幹姿勢の更新 */

    context.hpf_rot = context.filtered_rot - context.lpf_rot; /* High-Pass Filter */

    /* drot */
    context.lpf_drot = context.lpf_rot - context.prev_lpf_rot;
    context.prev_lpf_rot = context.lpf_rot;

#ifdef DEBUG_HOGE
    if(loop%500==0){
        std::cout << "[pr] rot=[" << (context.rot.array()*(180.0f/S_PI)).transpose() << "]" << std::endl
                  << "filtered_rot=[" << context.filtered_rot.transpose() << "]" << std::endl
                  << "lpf_rot=[" << (context.lpf_rot.array()*(180.0f/S_PI)).transpose() << "]" << std::endl
                  << "hpf_rot=[" <<  (context.hpf_rot.transpose() ) << "]"
                  << std::endl;
    }
#endif
}; /* updateRotContext */

#if 1
void PushRecover::modifyTrajectoryRot(const bool enable_modify, const bool on_ground,
                                      ModifyTrajectoryContext &context,
                                      TrajectoryElement<Vec3e> &ref_traj) const{
#if 0
    const double alpha = 0.3000;
    const double pgain = 1.0, dgain = 0.01*m_dt_i;
#elif 1
    /* TODO 負のゲインを試す */
    const double alpha = 0.2500;
    const double pgain = 0.6, dgain = 0.10*m_dt_i;
#elif 0
    const double alpha = 0.2500;
    const double pgain = 0.4, dgain = 0.05*m_dt_i;
#else
#endif
    //const Vec3 rot_offset = context.rot_offset.array()*alpha + (1.0-alpha)*(pgain * context.lpf_rot.array() + dgain * ((context.lpf_drot).array()));
    //const Vec3 rot_offset = Vec3::Zero();
    const Vec3 rot_offset = context.lpf_rot;
    //const Vec3 rot_offset_dummy = context.rot_offset.array(*)alpha + (1.0-alpha)*(pgain * context.lpf_rot.array() + dgain * ((context.lpf_drot).array()));


#ifndef DEBUG_HOGE
    if(loop%500==0){
        std::cout << "rot_offset = [" << ((180.0f/S_PI)*rot_offset[0]) << ", " << ((180.0f/S_PI)*rot_offset[1]) << ", "<< ((180.0f/S_PI)*rot_offset[2]) << "]" << std::endl;
        //std::cout << "rot_offset = " << rot_offset_dummy.transpose() << std::endl;
        //std::cout << " ========================= " << std::endl;
    }
#endif

    if(enable_modify){
        if( context.transition_state ){
            // transition stateのときはrot_offsetを補間する
            //ref_traj.body_p = ref_traj.body_p - (Vec3e)(context.rot_offset.array() * context.transition_gain);
            context.rot_offset = rot_offset.array() * context.transition_gain;

            context.transition_gain += context.transition_gain_diff;
            if( context.transition_gain > 1.0f ){
                context.transition_state = false;
            }
        }else{
            // transitionが終了している場合はそのままaddする
            //ref_traj.body_p = ref_traj.body_p - (Vec3e)(context.rot_offset);
            context.rot_offset = rot_offset;
        }
    }else{
        // Disable状態でtransition stateをリセットする
        context.transition_state = true;
        context.transition_gain = 0.0f;
    }
}; /* modifyTrajectoryRot */
#else
void PushRecover::modifyTrajectoryRot(const bool enable_modify, const bool on_ground,
                                      ModifyTrajectoryContext &context,
                                      TrajectoryElement<Vec3e> &ref_traj) const{
    const Vec3 v = Vec3(
                        ref_traj.body_p[0] - ref_traj.p[0],
                        ref_traj.body_p[1] - ref_traj.p[1],
                        traj_body_init[2]
                        );
    _MM_ALIGN16 float c[4],s[4];
    bodylink::sincos_ps( bodylink::F32vec4(context.hpf_rot[0],
                                           context.hpf_rot[1],
                                           context.hpf_rot[2], 0.0f),
                         (bodylink::v4sf*)s, (bodylink::v4sf*)c );
    const Mat3 body_R                  = rotateMat3<1>( c[1], s[1] )*rotateMat3<0>( c[0], s[0] );

    const Vec3 rot_offset = body_R*v - v;
    //context.rot_offset = rot_offset.array() * 0.900f;
    context.rot_offset = Vec3Zero();

#ifdef DEBUG_HOGE
    if(loop%500==0){
        std::cout << "rot_offset = " << rot_offset.transpose() << std::endl;
        std::cout << " ========================= " << std::endl;
    }
#endif

    if(enable_modify){
        if( context.transition_state ){
            // transition stateのときはrot_offsetを補間する
            ref_traj.body_p = ref_traj.body_p - (Vec3e)(context.rot_offset.array() * context.transition_gain);
            context.transition_gain += context.transition_gain_diff;
            if( context.transition_gain > 1.0f ){
                context.transition_state = false;
            }
        }else{
            // transitionが終了している場合はそのままaddする
            ref_traj.body_p = ref_traj.body_p - (Vec3e)(context.rot_offset);
        }
    }else{
        // Disable状態でtransition stateをリセットする
        context.transition_state = true;
        context.transition_gain = 0.0f;
    }
}; /* modifyTrajectoryRot */
#endif

void PushRecover::selectSimmodeControlValue(const int mode, hrp::Vector3 &basepos_modif, hrp::Vector3 &basepos_vel){
    switch(mode){
    case 1:
        basepos_modif = hrp::Vector3::Zero();
        basepos_vel   = hrp::Vector3::Zero();
        break;
    case 2: /* step forward */
        basepos_modif = hrp::Vector3(0.05f,0.0f,0.0f);
        basepos_vel = hrp::Vector3(0.1f,0.0f,0.0f);
        break;
    case 3: /* step left */
        basepos_modif = hrp::Vector3(0.0f,0.05f,0.0f);
        basepos_vel = hrp::Vector3(0.0f,0.2f,0.0f);
        break;
    case 4: /* step back right */
        basepos_modif = hrp::Vector3(-0.05f,-0.05f,0.0f);
        basepos_vel = hrp::Vector3(-0.1f,-0.1f,0.0f);
        break;
    case 5: /* step back right */
        basepos_modif = hrp::Vector3(-0.01f,-0.05f,0.0f);
        basepos_vel = hrp::Vector3(-0.05f,-0.1f,0.0f);
        break;
    case 6: /* step back right */
        basepos_modif = hrp::Vector3(-0.0f,-0.0f,0.0f);
        basepos_vel = hrp::Vector3(-0.05f,-0.1f,0.0f);
        break;
    case 7: /* step back right */
        basepos_modif = hrp::Vector3(-0.0f,-0.0f,0.0f);
        basepos_vel = hrp::Vector3(-0.1f,-0.01f,0.0f);
        break;
    case 8: /* step back right */
        basepos_modif = hrp::Vector3(0.0f,0.0f,0.0f);
        basepos_vel = hrp::Vector3(0.05f,0.10f,0.0f);
        break;
    case 9: /* step back right */
        basepos_modif = hrp::Vector3(0.0f,0.0f,0.0f);
        basepos_vel = hrp::Vector3(0.30f,0.01f,0.0f);
        break;
    default:
        basepos_modif = hrp::Vector3::Zero();
        basepos_vel = hrp::Vector3::Zero();
    }
};

void PushRecover::modifyFootHeight(const bool on_ground, ModifyTrajectoryContext &context, TrajectoryElement<Vec3e> &ref_traj){
    const double fz[2] = {
        m_prev_act_force_z[ ee_index_lr[EE_INDEX_LEFT] ], //left
        m_prev_act_force_z[ ee_index_lr[EE_INDEX_RIGHT] ] //right
    };
    const float foot_pz_ref[2] = {
        ref_traj.footl_p[2],
        ref_traj.footr_p[2]
    };
    const int swing_leg_lr[2] = {-1,1};

    for(int i=0;i<2;i++){
        /* swing legである、または接地判定が空中である場合は接地判定を行う */
        if( m_owpg.isSwingLeg(swing_leg_lr[i]) || (context.fz_contact[i]==0) ){
            /* swing legの接地判定。 シュミットトリガー  */
            if( context.fz_contact[i] == 1 ){
                context.fz_contact_z[i] *= 0.98;
                if( fz[i] < context.onlineWalkParam.fz_contact_threshold_lower ){
                    context.fz_contact[i] = 0;
                    if( foot_pz_ref[i] > context.fz_contact_z[i] ){
                        context.fz_contact_z[i] = 0.0f;
                    }
                }
            }else{
                context.fz_contact_z[i] *= 0.94;
                if( fz[i] > context.onlineWalkParam.fz_contact_threshold_upper ){
                    context.fz_contact[i] = 1;
                    context.fz_contact_z[i] = foot_pz_ref[i];
#ifdef DEBUG_HOGE
                    if(i==0){
                        std::cerr << "[pr] " << PRED << "contact" << PGRE << "LEFT" << PDEF << std::endl;
                    }else{
                        std::cerr << "[pr] " << PRED << "contact" << PBLU << "RIGHT" << PDEF << std::endl;
                    }
#endif
                }
            }
        }
    }

#define FOOT_Z_SELECT_MAX(x,y) ( (x)>(y)? (x) : (y) )
    switch( context.fz_transition_state ){
    case ModifyTrajectoryContext::FZ_TRANSITION_IDLE:
        if(on_ground){
            context.fz_transition_state = ModifyTrajectoryContext::FZ_TRANSITION_TO_READY;
            std::cout << "[pr]" <<__func__ << "() " << PRED << "move to Ready" << PDEF << std::endl;
        }
        context.fz_contact_z[0] = 0.0f;
        context.fz_contact_z[1] = 0.0f;
        break;
    case ModifyTrajectoryContext::FZ_TRANSITION_TO_READY:
        if(on_ground){
            context.fz_transition_gain += context.fz_transition_gain_diff;
            if( context.fz_transition_gain >= 1.0f ){
                context.fz_transition_gain = 1.0f;
                context.fz_transition_state = ModifyTrajectoryContext::FZ_TRANSITION_READY;
                std::cout << "[pr]" <<__func__ << "() " << PGRE << "Ready" << PDEF << std::endl;
            }
        }else{
            /* on AIR に戻ったらIDLEに戻る */
            context.fz_transition_state = ModifyTrajectoryContext::FZ_TRANSITION_TO_IDLE;
        }
        {
            ref_traj.footl_p[2] = FOOT_Z_SELECT_MAX(ref_traj.footl_p[2], context.fz_contact_z[0]*context.fz_transition_gain);
            ref_traj.footr_p[2] = FOOT_Z_SELECT_MAX(ref_traj.footr_p[2], context.fz_contact_z[1]*context.fz_transition_gain);
        }
        break;
    case ModifyTrajectoryContext::FZ_TRANSITION_TO_IDLE:
        if(on_ground){
            /* on ground に戻ったらREADYに戻す */
            context.fz_transition_state = ModifyTrajectoryContext::FZ_TRANSITION_TO_READY;
        }else{
            context.fz_transition_gain -= context.fz_transition_gain_diff;
            if( context.fz_transition_gain <= 0.0f ){
                context.fz_transition_gain = 0.0f;
                context.fz_transition_state = ModifyTrajectoryContext::FZ_TRANSITION_IDLE;
                std::cout << "[pr]" <<__func__ << "() " << PBLU << "IDLE" << PDEF << std::endl;
            }
        }
        {
            ref_traj.footl_p[2] = FOOT_Z_SELECT_MAX(ref_traj.footl_p[2], context.fz_contact_z[0]*context.fz_transition_gain);
            ref_traj.footr_p[2] = FOOT_Z_SELECT_MAX(ref_traj.footr_p[2], context.fz_contact_z[1]*context.fz_transition_gain);
        }
        break;
    case ModifyTrajectoryContext::FZ_TRANSITION_READY:
        if(on_ground){

        }else{
            context.fz_transition_state = ModifyTrajectoryContext::FZ_TRANSITION_TO_IDLE;
            std::cout << "[pr]" <<__func__ << "() " << PRED << "move to Idle" << PDEF << std::endl;
        }
        {
            ref_traj.footl_p[2] = FOOT_Z_SELECT_MAX(ref_traj.footl_p[2], context.fz_contact_z[0]);
            ref_traj.footr_p[2] = FOOT_Z_SELECT_MAX(ref_traj.footr_p[2], context.fz_contact_z[1]);
        }
        break;
    default:
        break;
    }
};

void PushRecover::interpretJoystickCommandandSend(const TimedFloatSeq &axes, const TimedBooleanSeq &buttons, JoyState &jstate, OnlinePatternGenerator* p_owpg) const{
    JoyCommand command;
    if(( buttons.data[1] == 1 ) && (jstate.enabled==false)){
        jstate.enabled = true;
    }else if( (buttons.data[2]==1) && (jstate.enabled==true) ){
        jstate.enabled = false;
    }
    if( buttons.data[0] == 1 ){
        jstate.keep_idle = true;
    }else{
        jstate.keep_idle = false;
    }

    command.x = -axes.data[1];
    command.y = -axes.data[0];

    if( jstate.enabled && loop%10==0 ){
        p_owpg->command(command);
    }

    jstate.prev_command = command;

#ifdef DEBUG_HOGE
    if( loop%200==0 ){
        std::cout << "[pr] joycommand [" << command.x << ", " << command.y << std::endl;
    }
#endif
};

void PushRecover::interpretJoystickCommandandSend(const TimedFloatSeq &axes, const TimedBooleanSeq &buttons, JoyState &jstate, RobustOnlinePatternGenerator* p_owpg) const{
    JoyCommand command;
    const size_t button_length = buttons.data.length();
    if( button_length==8 ){
        /* for flight stick */
        if(( buttons.data[1] == 1 ) && (jstate.enabled==false)){
            jstate.enabled = true;
        }else if( (buttons.data[2]==1) && (jstate.enabled==true) ){
            jstate.enabled = false;
        }
        if( buttons.data[0] == 1 ){
            jstate.keep_idle = true;
        }else{
            jstate.keep_idle = false;
        }

        command.x = -axes.data[1];
        command.y = -axes.data[0];

        if( jstate.enabled && loop%10==0 ){
            p_owpg->command(command);
        }

        jstate.prev_command = command;

        //std::cout << "[pr] flight command.x,y=[" << command.x << ", " << command.y << "], en=" << jstate.enabled << ", keep=" << jstate.keep_idle << std::endl;
    }else if( button_length==11 ){
        /* for PS3 and XBOX joy */
        if(( buttons.data[1] == 1 ) && (jstate.enabled==false)){ // B Botton on XBOX
            jstate.enabled = true;
        }else if( (buttons.data[0]==1) && (jstate.enabled==true) ){ //A Botton on XBOX
            jstate.enabled = false;
        }

        if( buttons.data[5]==1 ){ // RB Botton on XBOX
            jstate.keep_idle = true;
        }else{
            jstate.keep_idle = false;
        }

        {
            const double thre = 0.0305;
            const double d = -axes.data[1];
            command.x = (d>=thre)?(d-thre):((d<=-thre)?(d+thre):0.0);
        }
        {
            const double thre = 0.0305;
            const double d = -axes.data[0];
            command.y = (d>=thre)?(d-thre):((d<=-thre)?(d+thre):0.0);
        }

        if( jstate.enabled && loop%10==0 ){
            p_owpg->command(command);
        }

        jstate.prev_command = command;

        //std::cout << "[pr] ps3 command.x,y=[" << command.x << ", " << command.y << "], en=" << jstate.enabled << ", keep=" << jstate.keep_idle << std::endl;
    }else{
        if( loop%5000==0 ){
            std::cout << PRED << "[pr] Warning: button length[" << button_length << "] is not undefined. It seems no joystick is connected." << PDEF << std::endl;
        }
    }

#ifdef DEBUG_HOGE
    if( loop%200==0 ){
        std::cout << "[pr] joycommand [" << command.x << ", " << command.y << std::endl;
    }
#endif
};

extern "C"
{
    void PushRecoverInit(RTC::Manager* manager);
};

#define PRINTVEC3(v,f)

#endif // PUSHRECOVER_H
