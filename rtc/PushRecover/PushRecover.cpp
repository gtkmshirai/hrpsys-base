// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
/*!
 * @file  PushRecover.cpp
 * @brief Push recovery controller
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "PushRecover.h"
#include <hrpUtil/MatrixSolvers.h>
#include <hrpModel/Sensor.h>

#include <boost/static_assert.hpp>

#define print_traj(v) \
    std::cout << #v << ".p= [" << (v.p).transpose() << "]" << std::endl;\
    std::cout << #v << ".body_p= [" << (v.body_p).transpose() << "]" << std::endl;\
    std::cout << #v << ".footl_p= [" << (v.footl_p).transpose() << "]" << std::endl;\
    std::cout << #v << ".footr_p= [" << (v.footr_p).transpose() << "]" << std::endl;\
    std::cout << #v << ".dp= [" << (v.dp).transpose() << "]" << std::endl;\
    std::cout << #v << ".body_dp= [" << (v.body_dp).transpose() << "]" << std::endl

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* pushrecover_spec[] =
  {
    "implementation_id", "PushRecover",
    "type_name",         "PushRecover",
    "description",       "Push Recover",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    "conf.default.simmode", "0",
    ""
  };
// </rtc-template>

PushRecover::PushRecover(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      //Inport
      m_qRefIn("qRef", m_qRef),
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_zmpIn("zmpIn", m_zmp),
      m_rateIn("rate", m_rate),
      m_rpyIn("rpy", m_rpy),
      m_accIn("acc", m_acc),
      m_emergencySignalIn("emergencySignal", m_emergencySignal),
      m_accRefIn("accRefIn", m_accRef),
      m_contactStatesIn("contactStatesIn", m_contactStates),
      m_controlSwingSupportTimeIn("controlSwingSupportTimeIn", m_controlSwingSupportTime),
      m_walkingStatesIn("walkingStatesIn", m_walkingStates),
      m_sbpCogOffsetIn("sbpCogOffsetIn", m_sbpCogOffset),
      m_joyaxesIn("joyaxes", m_joyaxes),
      m_joybuttonsIn("joybuttons", m_joybuttons),
      m_qRefOut("q", m_qRef),
      m_tauRefOut("tau", m_tauRef),
      m_wRefOut("wc", m_wRef),
      m_wheel_brakeOut("wb", m_wheel_brake),
      m_zmpOut("zmpOut", m_zmp),
      m_basePosOut("basePosOut", m_basePos),
      m_baseRpyOut("baseRpyOut", m_baseRpy),
      m_basePoseOut("basePoseOut", m_basePose),
      m_accRefOut("accRefOut", m_accRef),
      m_contactStatesOut("contactStatesOut", m_contactStates),
      m_controlSwingSupportTimeOut("controlSwingSupportTimeOut", m_controlSwingSupportTime),
      m_walkingStatesOut("walkingStatesOut", m_walkingStates),
      m_sbpCogOffsetOut("sbpCogOffsetOut", m_sbpCogOffset),
      m_PushRecoverServicePort("PushRecoverService"),
      m_dt_gen(0.002f),
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
      m_owpg(0.001f, Zc, LegIKParam::InitialLfoot_p[1], traj_body_init, LegIKParam::InitialLfoot_p, LegIKParam::InitialRfoot_p, OnlinePatternGenerator::func<LegIKParam, LEG_IK_TYPE>), /* 500Hz*/
#else
      m_owpg(m_dt_gen, Zc, LegIKParam::InitialLfoot_p[1], traj_body_init, LegIKParam::InitialLfoot_p, LegIKParam::InitialRfoot_p, RobustOnlinePatternGenerator::func<LegIKParam, LEG_IK_TYPE>), /* 500Hz*/
#endif
#if defined(USE_APECF)
      m_abs_est(0.002f),
#endif
      m_owpg_cstate(&ctx,&cty),
      rate_matcher(500,1000),
      ee_params(2), /* Default number of End Effector is 2 */
// </rtc-template>
      m_simmode(0),
      m_generator_select(1),
      m_debugLevel(0),
      m_expectedJointNum(12),
      //dlogger(40*500)
      dlogger(40*500, "datalog_pr.dat")
{
    m_service0.pushrecover(this);
    emergencyStopReqFlag = false;
    m_prev_owpg_isComplete = false;
    InitContext(&ctx);
    InitContext(&cty);
}



PushRecover::~PushRecover()
{
    std::cout << "[pr] Destructor" << std::endl;
}



RTC::ReturnCode_t PushRecover::onInitialize()
{
  std::cout << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  std::cout << "[" << m_profile.instance_name << "] onInitialize() 1, debugLevel=" << m_debugLevel << std::endl;
  bindParameter("simmode", m_simmode, "0");
  std::cout << "[" << m_profile.instance_name << "] onInitialize() 1, simmode=" << m_simmode << std::endl;
  bindParameter("generator_select", m_generator_select, "1");
  std::cout << "[" << m_profile.instance_name << "] onInitialize() 1, generator_select=" << m_generator_select << std::endl;
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("rate", m_rateIn);
  addInPort("rpy", m_rpyIn);
  addInPort("acc", m_accIn);
  addInPort("basePosIn", m_basePosIn);
  addInPort("baseRpyIn", m_baseRpyIn);
  addInPort("zmpIn", m_zmpIn);
  addInPort("emergencySignal", m_emergencySignalIn);

  addInPort("accRefIn", m_accRefIn);
  addInPort("contactStatesIn", m_contactStatesIn);
  addInPort("controlSwingSupportTimeIn", m_controlSwingSupportTimeIn);
  addInPort("walkingStatesIn", m_walkingStatesIn);
  addInPort("sbpCogOffsetIn", m_sbpCogOffsetIn);

  addInPort("joyaxes", m_joyaxesIn);
  addInPort("joybuttons", m_joybuttonsIn);
#if 0
  m_joyaxes.data.length(6); //TODO
  m_joybuttons.data.length(8); //TODO
#else
  m_joyaxes.data.length(1); //TODO
  m_joybuttons.data.length(1); //TODO
#endif

  // Set OutPort buffer
  addOutPort("q", m_qRefOut);
  addOutPort("tau", m_tauRefOut);
  m_tauRef.data.length(12); //TODO
  addOutPort("zmpOut", m_zmpOut);
  addOutPort("basePosOut", m_basePosOut);
  addOutPort("baseRpyOut", m_baseRpyOut);
  addOutPort("basePoseOut", m_basePoseOut);
  addOutPort("accRefOut", m_accRefOut);
  addOutPort("contactStatesOut", m_contactStatesOut);
  addOutPort("controlSwingSupportTimeOut", m_controlSwingSupportTimeOut);
  addOutPort("walkingStatesOut", m_walkingStatesOut);
  addOutPort("sbpCogOffsetOut", m_sbpCogOffsetOut);
  // Set OutPort buffer

  // Set service provider to Ports
  m_PushRecoverServicePort.registerProvider("service0", "PushRecoverService", m_service0);

  // Set service consumers to Ports
  std::cout << "[" << m_profile.instance_name << "] onInitialize() 2" << std::endl;
  // Set CORBA Service Ports
  addPort(m_PushRecoverServicePort);

  std::cout << "[" << m_profile.instance_name << "] onInitialize() 3" << std::endl;
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());
  m_dt_i = 1.0/m_dt;

  std::cout << "[" << m_profile.instance_name << "] onInitialize() 4" << std::endl;
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

#ifdef USE_DATALOG
  //slogger = new SimpleLogger();
    // printf("slogger ptr=%p, sizeof=%d\n", ((void*)slogger), sizeof(SimpleLogger));
    // for(int i=0;i<sizeof(SimpleLogger);i++){
    //     if(i%10==0){
    //         printf("[%d] ",i);
    //     }
    //     printf("%#2x", *(((unsigned char*)slogger)+i));
    //     if(i%10==9){
    //         printf("\n");
    //     }else{
    //         printf(", ");
    //     }
    // }
    // printf("\n");
#endif

  //parameters for internal robot model
  std::cout << "Loading Model=[" << prop["model"].c_str() << std::endl;

  {
      m_robot = hrp::BodyPtr(new hrp::Body());
      if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                                   CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                   )){
          std::cerr << "PR failed to load model[" << prop["model"] << "] in "
                    << m_profile.instance_name << std::endl;
          return RTC::RTC_ERROR;
      }
 }
  {
      m_robot_current = hrp::BodyPtr(new hrp::Body());
      if (!loadBodyFromModelLoader(m_robot_current, prop["model"].c_str(),
                                   CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                   )){
          std::cerr << "PR failed to load model[" << prop["model"] << "] in "
                    << m_profile.instance_name << std::endl;
          return RTC::RTC_ERROR;
      }
  }

  m_pleg[EE_INDEX_LEFT] = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot_current, m_robot_current->rootLink(), m_robot_current->link("LLEG_JOINT5"), m_dt, false, std::string(m_profile.instance_name)));
  m_pleg[EE_INDEX_RIGHT] = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot_current, m_robot_current->rootLink(), m_robot_current->link("RLEG_JOINT5"), m_dt, false, std::string(m_profile.instance_name)));

  for(int i=0;i<2;i++){
      if(m_pleg[i]==NULL){
          std::cerr << "PR failed to create jointPathEx[" << i << "]" << std::endl;
      return RTC::RTC_ERROR;
      }
  }

  std::cout << "[" << m_profile.instance_name << "] onInitialize() 5" << std::endl;
  /* Start Setting Force Sensor Port */
  // Setting for wrench data ports (real + virtual)
  std::vector<std::string> fsensor_names;
  //   find names for real force sensors
  int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
  for (unsigned int i=0; i<npforce; i++){
      fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
  }
  std::cout << "[" << m_profile.instance_name << "] onInitialize() 6" << std::endl;
  //   add ports for all force sensors
  int nforce  = npforce;
  m_force.resize(nforce);
  m_forceIn.resize(nforce);
  m_ref_force.resize(nforce);
  m_ref_forceIn.resize(nforce);
  m_ref_forceOut.resize(nforce);
  std::cerr << "[" << m_profile.instance_name << "] force sensor ports" << std::endl;
  for (unsigned int i=0; i<nforce; i++){
      // actual inport
      m_forceIn[i] = new RTC::InPort<TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
      m_force[i].data.length(6);
      registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
      // ref inport
      m_ref_force[i].data.length(6);
      for (unsigned int j=0; j<6; j++) m_ref_force[i].data[j] = 0.0;
      m_ref_forceIn[i] = new RTC::InPort<TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"In").c_str(), m_ref_force[i]);
      registerInPort(std::string("ref_"+fsensor_names[i]+"In").c_str(), *m_ref_forceIn[i]);

      // ref outport
      //m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string(sensor_names[i]).c_str(), m_force[i]);
      m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string("ref_" + fsensor_names[i]+"Out").c_str(), m_ref_force[i]);
      registerOutPort(std::string("ref_"+fsensor_names[i]+"Out").c_str(), *m_ref_forceOut[i]);

      std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
  }
  for (unsigned int i=0; i<m_forceIn.size(); i++){
      abs_forces.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
      abs_moments.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
  }
  /* Finish Setting Force Sensor Port */

  // setting from conf file
  // rleg,TARGET_LINK,BASE_LINK
  coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
  const size_t prop_num = 10;

#if 0
  std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << std::endl;
  for(int k=0; k<end_effectors_str.size(); k++){
      std::cout << "[pr] eeprop[" << k << "]" << end_effectors_str[k] << std::endl;
  }
  std::cout << MAKE_CHAR_COLOR_DEFAULT << std::endl;
#endif

  if (end_effectors_str.size() > 0) {
      const size_t num = end_effectors_str.size()/prop_num;
      /* Resize ee_params vector */
      if(ee_params.size() < num){
          ee_params.resize(num);
      }

      for (size_t i = 0; i < num; i++) {
          std::string ee_name, ee_target, ee_base;
          coil::stringTo(ee_name,   end_effectors_str[i*prop_num].c_str());
          coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
          coil::stringTo(ee_base,   end_effectors_str[i*prop_num+2].c_str());
          hrp::Vector3 ee_localp;
          for (size_t j = 0; j < 3; j++) {
              coil::stringTo(ee_localp(j), end_effectors_str[i*prop_num+3+j].c_str());
          }
          printf("[pr] oninitialize ee_localp:\n");
          std::cout << "[pr] " << ee_name << ": " << ee_target << ": " << std::endl;
          PRINTVEC3(ee_localp,true);
          double tmp_q[4];
          for (int j = 0; j < 4; j++ ) {
              coil::stringTo(tmp_q[j], end_effectors_str[i*prop_num+6+j].c_str());
          }
          hrp::Matrix33 ee_localR = Eigen::AngleAxis<double>(tmp_q[3], hrp::Vector3(tmp_q[0], tmp_q[1], tmp_q[2])).toRotationMatrix(); // rotation in VRM
          /* link endeffector name and index */
          ee_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
          /* Save End Effector name indexed */
          {
              vector<std::string>::iterator it;
              std::string serching_fsensor_name;
              if(ee_name=="lleg"){
                  serching_fsensor_name = "lfsensor";
              }else if(ee_name=="rleg"){
                  serching_fsensor_name = "rfsensor";
              }else if(ee_name=="larm"){
                  serching_fsensor_name = "lasensor";
              }else if(ee_name=="rarm"){
                  serching_fsensor_name = "rasensor";
              }else{
                  serching_fsensor_name = "none";
              }
              std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "serching_fsensor_name=" << serching_fsensor_name << MAKE_CHAR_COLOR_DEFAULT << std::endl;
              /* serch for if expected sensor is exists */
              if(serching_fsensor_name!="none"){
                  it = find(fsensor_names.begin(),fsensor_names.end(), serching_fsensor_name);
                  if(it==fsensor_names.end()){
                      std::cout << "[" << m_profile.instance_name << "] Error " << serching_fsensor_name << " not found" << std::endl;
                      return RTC::RTC_ERROR;
                  }else{
                      ee_params[ee_index_map[ee_name]].fsensor_name = *it;
                  }
              }else{
                  ee_params[ee_index_map[ee_name]].fsensor_name = "none";
              }
              ee_params[ee_index_map[ee_name]].ee_name   = ee_name;
              ee_params[ee_index_map[ee_name]].ee_target = ee_target;
              ee_params[ee_index_map[ee_name]].ee_base   = ee_base;
              ee_params[ee_index_map[ee_name]].ee_localp = ee_localp;
              ee_params[ee_index_map[ee_name]].ee_localR = ee_localR;
              ee_params[ee_index_map[ee_name]].act_contact_state = false;
          }
      }
      {
          /* left, rightの順でee_index_mapにアクセスするようのindexを先に作っておく */
          const std::string legs_ee_name[2]={"lleg","rleg"};
          for (size_t i = 0; i < 2; i++) {
              ee_index_lr[i] = ee_index_map[legs_ee_name[i]];
              std::cout << "[pr] ee_index_lr[" << i << "] = " << ee_index_map[legs_ee_name[i]];
          }
      }
  }
  /* Finish Setting EndEffector from config file */

  /* clear Emergency Stop Request flag */
  emergencyStopReqFlag = false;

  /* Initialize Acceleration Reference */
  m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;
  prev_imu_sensor_pos = hrp::Vector3::Zero();
  prev_imu_sensor_vel = hrp::Vector3::Zero();


  /* Initialize Reference Joint Angle Array */
  //ref_q = new double[m_robot->numJoints()];
  //m_prev_ref_q = new double[m_robot->numJoints()];
  for(int i=0;i<m_robot->numJoints();i++){
      m_ref_q_store[i] = m_prev_ref_q[i] = m_ref_q[i] = 0.0;
  }

  std::cout << "[" << m_profile.instance_name << "] calcik start" << std::endl;
#if defined(__INTEL_COMPILER)||defined(__ICC)
  BOOST_STATIC_ASSERT( __alignof__(BodyIKMethod) == 16 );
  m_pIKMethod          = new BodyIKMethod( 0.0f, Zc );
#elif defined(__GNUC__)
  //m_pIKMethod          = new BodyIKMethod( 0.0f, Zc - g_CoG_offset[2] );
  m_pIKMethod          = new BodyIKMethod( traj_body_init[0], traj_body_init[2] );
#endif
  /* Initialize Default joint angle on PR_READY state */
  {
      _MM_ALIGN16 float target_joint_angle[12];
      _MM_ALIGN16 float pre_joint_angle[12];
#if defined(__INTEL_COMPILER)||defined(__ICC)
      _MM_ALIGN16 Mat3 body_R = Mat3::identity();
#elif defined(__GNUC__)
      const Mat3 body_R = Mat3::Identity();
#endif
      //const float foot_l_pitch = 0.0f + LegIKParam::InitialLfoot_pitch;
      //const float foot_r_pitch = 0.0f + LegIKParam::InitialRfoot_pitch;
      const float foot_l_pitch = 0.0f;
      const float foot_r_pitch = 0.0f;
      const float foot_l_roll = 0.0f, foot_r_roll = 0.0f;
      m_pIKMethod->calcik_ini(body_R,
                              body_p_default_offset,
                              Vec3Zero(),
                              LegIKParam::InitialLfoot_p,
                              Vec3Zero(),
                              LegIKParam::InitialRfoot_p,
                              LegIKParam::InitialLfoot_R,
                              LegIKParam::InitialRfoot_R,
                              foot_l_pitch,
                              foot_r_pitch,
                              foot_l_roll,
                              foot_r_roll,
                              target_joint_angle );
//       _MM_ALIGN16 Vec3 body_p = m_pIKMethod->calcik(body_R,
//                                                     body_p_default_offset,
// #if 0
//                                                     LegIKParam::InitialLfoot_p - default_zmp_offset_l,
//                                                     LegIKParam::InitialRfoot_p - default_zmp_offset_r,
// #else
//                                                     LegIKParam::InitialLfoot_p,
//                                                     LegIKParam::InitialRfoot_p,
// #endif
//                                                     foot_l_pitch,
//                                                     foot_r_pitch,
//                                                     target_joint_angle );
#if ROBOT==0
      for(int i=0;i < 12; i++){
          m_ready_joint_angle[i] = target_joint_angle[i];
      }
#elif ROBOT==1
      for(int i=0;i < 6; i++){
          /* m_robot of L1 starts from right leg. */
          m_ready_joint_angle[i+6] = target_joint_angle[i];
          m_ready_joint_angle[i]   = target_joint_angle[i+6];
      }
#elif ROBOT==2
      for(int i=0;i < 6; i++){
          /* m_robot of L1 starts from right leg. */
          m_ready_joint_angle[i+6] = target_joint_angle[i];
          m_ready_joint_angle[i]   = target_joint_angle[i+6];
      }
#else
#error "PushRecover setting m_robot->joint(i)-q. Undefined ROBOT Type"
#endif
  }
  std::cout << "[" << m_profile.instance_name << "] calcik end" << std::endl;

  /* Initialize transition interpolator to interpolate from idle state to push recover ready state */
  {
      const int interpolator_dimension = 1;
      const double average_velocity = 1.0;
      transition_interpolator = new interpolator(interpolator_dimension, m_dt, interpolator::HOFFARBIB, average_velocity);
      transition_interpolator->setName(std::string(m_profile.instance_name)+" transition_interpolator");
      transition_interpolator_ratio = 1.0;
  } /* Finish initializing transition interpolator */

  /* Initialize CoG velocity IIR filter */
  act_cogvel_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, m_dt, hrp::Vector3::Zero())); // [Hz]
  ref_zmp_modif_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, m_dt, hrp::Vector3::Zero())); // [Hz]
  ref_basePos_modif_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, m_dt, hrp::Vector3::Zero())); // [Hz]

  /* Initialize PR trajectory generator related values */
  trajectoryReset();

  dlog_save_flag = false;
  gettimeofday(&stv,NULL); /* save the time of initialized */

  prev_act_zmp      = hrp::Vector3::Zero();
  m_prev_act_root_pos = hrp::Vector3::Zero();
  m_prev_act_cog      = hrp::Vector3::Zero();
  m_prev_act_cog_base = hrp::Vector3::Zero();
  m_prev_ref_cog      = hrp::Vector3::Zero();
  m_ref_cogvel        = hrp::Vector3::Zero();
  m_prev_act_root_pos_base = hrp::Vector3::Zero();
  m_prev_act_cogvel   = hrp::Vector3::Zero();
  act_base_rpy      = hrp::Vector3::Zero();
  m_ref_zmp         = hrp::Vector3::Zero();
  prev_ref_zmp      = hrp::Vector3::Zero();
  ref_basePos       = hrp::Vector3::Zero();
  ref_zmp_modif     = hrp::Vector3::Zero();
  m_ref_basePos_modif = hrp::Vector3::Zero();

  m_prev_act_foot_origin_rot = hrp::Matrix33::Identity();
  input_baseRot              = hrp::Matrix33::Identity();
  ref_baseRot                = hrp::Matrix33::Identity();

  /* Initialize PushDetector */
  pushDetector_state = PD_DISABLE;
  pushDetectParam.diff_margin_threshold_x = 70.0*70.0;
  pushDetectParam.diff_margin_threshold_y = (40.0+80.0)*(40.0+80.0);
  pushDetectParam.body_compliance_k1      = 0.03;
  pushDetectParam.body_compliance_k2      = -0.03;
  pushDetectParam.body_compliance_k3      = 0.1;
  pushDetectParam.x_gain_0                = 1.0;
  pushDetectParam.x_gain_1                = 1.0;
  pushDetectParam.dx_gain_0               = 1.0;
  pushDetectParam.dx_gain_1               = 1.0;

  m_mg  = m_robot->totalMass() * 9.80;
  m_mg2 = m_robot->totalMass() * (9.80/2.0);

  loop = 1;

#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
  m_modify_rot_context.copyWalkParam( &m_owpg );
//#error "m_owpg is used"
#else
  //m_owpg.setModifyThre(20e-3);
  m_modify_rot_context.copyWalkParam( &m_owpg );
#endif

  //RTC::Properties& prop = getProperties();
  m_expectedJointNum = std::atoi(prop["expectedJointNum"].c_str());
  std::cout << "expectedJointNum = " << m_expectedJointNum << std::endl;

  /* Check the validity by joint num and robot name */
  if(m_robot->numJoints()!=m_expectedJointNum){
      std::cout << "[" << m_profile.instance_name << "] number of joint is expected to be " << m_expectedJointNum << "." << std::endl;
      return RTC::RTC_ERROR;
  }
  if(m_robot->name() != LegIKParam::name){
      std::cout << "[" << m_profile.instance_name << "] m_robot->name("<<m_robot->name()<<") does not match with LegIKParam::name(" << LegIKParam::name <<  ")"<< std::endl;
      return RTC::RTC_ERROR;
  }
  if(m_robot->name() != RobotConfiguration::robotname){
      std::cout << "[" << m_profile.instance_name << "] m_robot->name("<<m_robot->name()<<") does not match with RobotConfiguration::robotname(" << RobotConfiguration::robotname <<")"<< std::endl;
      return RTC::RTC_ERROR;
  }

  /* Initialize Wheel Controller if robot has wheel leg. */
  if(m_robot->name() == "L1W"){
      std::cout << "[" << m_profile.instance_name << "] Robot name matched L1W. Generating wheel controller." << std::endl;
      const int wheel_num = 2;
      /* wc (Wheel Command) port exists only if robot has wheeled leg */
      addOutPort("wc", m_wRefOut);
      m_wRef.data.length(wheel_num);
      addOutPort("wb", m_wheel_brakeOut);
      m_wheel_brake.data.length(wheel_num);
      m_wheel_ctrl = std::make_shared<WheelLeg::WheelController>(wheel_num);
      m_wheel_ctrl->activate();
      std::cout << "[" << m_profile.instance_name << "] Reading wheel controller initial gain from conf files." << std::endl;
      double pgain,dgain;
      coil::stringTo(pgain, prop["wc_pgain"].c_str());
      coil::stringTo(dgain, prop["wc_dgain"].c_str());
      std::cout << "[" << m_profile.instance_name << "] wheel_ctrl.pgain = " << pgain << std::endl;
      std::cout << "[" << m_profile.instance_name << "] wheel_ctrl.dgain = " << dgain << std::endl;
      m_wheel_ctrl->setPgain(pgain);
      m_wheel_ctrl->setDgain(dgain);
  }else{
      std::cout << "[" << m_profile.instance_name << "] Robot name does not match L1W." << std::endl;
      m_wheel_ctrl = std::make_shared<WheelLeg::WheelController>(2);
      m_wheel_ctrl->deactivate();
      std::cout << "[" << m_profile.instance_name << "] Robot name does not matched L1W. Not generate Wheel controller." << std::endl;
  }

  std::cout << "[" << m_profile.instance_name << "] onInitialize() Finished." << std::endl;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t PushRecover::onFinalize()
{
    std::cout << "[pr] onFinalize" << std::endl;
    delete transition_interpolator;
    delete m_pIKMethod;
#ifdef USE_DATALOG
    //delete slogger;
#endif
    return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t PushRecover::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PushRecover::onActivated(RTC::UniqueId ec_id)
{
    std::cout << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;

    m_current_control_state = PR_IDLE;

    return RTC::RTC_OK;
};

RTC::ReturnCode_t PushRecover::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    Guard guard(m_mutex);
    if(m_current_control_state != PR_IDLE){
        m_current_control_state = PR_IDLE;
        const double tmp_ratio = 0.0;
        transition_interpolator->go(&tmp_ratio, m_dt, true); // sync in one controller loop
    }
    return RTC::RTC_OK;
};

void PushRecover::updateInputData(const bool shw_msg_flag){
#define DEBUG_INPUT_PRINT(x)  {if(shw_msg_flag) std::cout << "[pr] " << #x << "=" << x << std::endl;}
#define DEBUG_INPUT_PRINT_I(x,i)  {if(shw_msg_flag) std::cout << "[pr] " << #x << "[" << i << "]=" << x##[i] << std::endl;}

    if(shw_msg_flag){
        std::cout << "[pr] "  << __func__ << std::endl;
    }

    try {
        for (unsigned int i=0; i<m_forceIn.size(); i++){
            if ( m_forceIn[i]->isNew() ) {
                m_forceIn[i]->read();
                DEBUG_INPUT_PRINT(m_force[i].data[0]);
                DEBUG_INPUT_PRINT(m_force[i].data[1]);
                DEBUG_INPUT_PRINT(m_force[i].data[2]);
            }
            if ( m_ref_forceIn[i]->isNew() ) {
                m_ref_forceIn[i]->read();
                DEBUG_INPUT_PRINT(m_ref_force[i].data[0]);
                DEBUG_INPUT_PRINT(m_ref_force[i].data[1]);
                DEBUG_INPUT_PRINT(m_ref_force[i].data[2]);
            }
        }
        if (m_basePosIn.isNew()) {
            m_basePosIn.read();
            input_basePos(0) = m_basePos.data.x;
            input_basePos(1) = m_basePos.data.y;
            input_basePos(2) = m_basePos.data.z;
            DEBUG_INPUT_PRINT(input_basePos(0));
            DEBUG_INPUT_PRINT(input_basePos(1));
            DEBUG_INPUT_PRINT(input_basePos(2));
        }
        if (m_baseRpyIn.isNew()) {
            m_baseRpyIn.read();
            input_baseRot = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
            DEBUG_INPUT_PRINT(m_baseRpy.data.r);
            DEBUG_INPUT_PRINT(m_baseRpy.data.p);
            DEBUG_INPUT_PRINT(m_baseRpy.data.y);
        }
        if (m_zmpIn.isNew()) {
            m_zmpIn.read();
            input_zmp(0) = m_zmp.data.x;
            input_zmp(1) = m_zmp.data.y;
            input_zmp(2) = m_zmp.data.z;
            DEBUG_INPUT_PRINT(input_zmp(0));
            DEBUG_INPUT_PRINT(input_zmp(1));
            DEBUG_INPUT_PRINT(input_zmp(2));
        }
        if (m_rateIn.isNew()){
            m_rateIn.read();
        }
        if (m_rpyIn.isNew()) {
            m_rpyIn.read();
            DEBUG_INPUT_PRINT(m_rpy.data.r);
            DEBUG_INPUT_PRINT(m_rpy.data.p);
            DEBUG_INPUT_PRINT(m_rpy.data.y);
        }
        if (m_accIn.isNew()){
            m_accIn.read();
        }
        if (m_qCurrentIn.isNew()) {
            m_qCurrentIn.read();
        }
        if (m_qRefIn.isNew()) {
            m_qRefIn.read();
        }
        if (m_emergencySignalIn.isNew()){
            m_emergencySignalIn.read();
            std::cout << "[" << m_profile.instance_name << "] emergencySignal is set!" << std::endl;
        }
        if (m_accRefIn.isNew()){
            m_accRefIn.read();
        }
        if (m_contactStatesIn.isNew()){
            m_contactStatesIn.read();
        }
        if (m_controlSwingSupportTimeIn.isNew()){
            m_controlSwingSupportTimeIn.read();
        }
        if (m_walkingStatesIn.isNew()){
            m_walkingStatesIn.read();
        }
        if (m_sbpCogOffsetIn.isNew()){
            m_sbpCogOffsetIn.read();
        }
        if (m_joyaxesIn.isNew()){
            m_joyaxesIn.read();
            if((loop%200==0) && m_debugLevel==5){
                std::cout << "[pr] joy a=[";
                for( int i=0;i<m_joyaxes.data.length(); i++){
                    printf("%1.3lf", (double)m_joyaxes.data[i]);
                    if( i == m_joyaxes.data.length()-1 ){
                        printf("]\n");
                    }else{
                        printf(", ");
                    }
                }
            }
        }
        if (m_joybuttonsIn.isNew()){
            m_joybuttonsIn.read();
            if((loop%200==0) && m_debugLevel==5){
                std::cout << "[pr] joy a=[";
                for( int i=0;i<m_joybuttons.data.length(); i++){
                    printf("%1.3lf", (double)m_joybuttons.data[i]);
                    if( i == m_joybuttons.data.length()-1 ){
                        printf("]\n");
                    }else{
                        printf(", ");
                    }
                }
            }
        }
    }catch(int e){
        std::cerr << "[pr] " << PRED << "input PORT failed[" << e << "]" << PDEF << std::endl;
    }catch(...){
        std::cerr << "[pr] " << PRED << "input PORT failed" << PDEF << std::endl;
    }

    {
        if(shw_msg_flag){
            std::cout << "[pr] accRef=";
            printf("%lf, %lf, %lf\n", m_accRef.data.ax, m_accRef.data.ay, m_accRef.data.az);
            std::cout << "[pr] controlSwingSupportTime=";
            printf("%lf, %lf\n",m_controlSwingSupportTime.data[0],m_controlSwingSupportTime.data[1]);
            if(m_walkingStates.data){
                std::cout << "[pr] walkingstates=true" << std::endl;
            }else{
                std::cout << "[pr] walkingstates=false" << std::endl;
            }
            std::cout << "[pr] sbpCogOffsetIn=";
            printf("%lf, %lf, %lf\n",m_sbpCogOffset.data.x,m_sbpCogOffset.data.y,m_sbpCogOffset.data.z);
        }
    }

    if(shw_msg_flag){
        std::cout <<std::endl;
    }
#undef DEBUG_INPUT_PRINT
#undef DEBUG_INPUT_PRINT_I
};

hrp::Vector3 PushRecover::updateEstimatedInputData(void){
    //com_dph = hrp::Vector3(Zc * m_rpy.data.p, Zc * m_rpy.data.r, 0.0) + com_dph_base;
    //return hrp::Vector3(Zc * m_rpy.data.p, Zc * m_rpy.data.r, 0.0);
    return hrp::Vector3(Zc * act_base_rpy[1], -Zc * act_base_rpy[0], 0.0);
};

void PushRecover::updateEstimatedOutputData(void){
};

void PushRecover::setTargetDataWithInterpolation(void){
    const PushRecoveryState tmp_state = m_current_control_state;
    bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
    if (!is_transition_interpolator_empty) {
        /* Interpolation is currently working */
        transition_interpolator->get(&transition_interpolator_ratio, true);
    } else {
        /* Interpolation is not currently working */
        if(m_current_control_state == PR_TRANSITION_TO_READY){
            m_current_control_state = PR_READY;
            transition_interpolator_ratio = 1.0; /* use controller output */

            /* Initialize PR trajectory generator related values */
            trajectoryReset();
            stpf.reset();

            /* Set Initial Base Frame Reference Position */
            m_robot->rootLink()->p = hrp::Vector3(traj_body_init[0], traj_body_init[1], traj_body_init[2]+LegIKParam::InitialLfoot_p[2]);
            /* Set Initial Base Frame Reference Rotation */

            /* Transition to state PR_READY needs to set default values */
            for ( int i = 0; i < m_robot->numJoints(); i++ ) {
                //m_robot->joint(i)->q = g_ready_joint_angle[i];
                m_robot->joint(i)->q = m_ready_joint_angle[i];
            }
#ifdef DEBUG_HOGE
            std::cout << "PR_TRANSITION_TO_READY completed." << std::endl;
#endif
        }else if(m_current_control_state == PR_TRANSITION_TO_IDLE){
            transition_interpolator_ratio = 0.0; /* use input as output */
            m_current_control_state = PR_IDLE;

            /* Transition to state PR_READY needs to set default values */
            for ( int i = 0; i < m_robot->numJoints(); i++ ) {
                m_robot->joint(i)->q = m_qRef.data[i];
            }
            /* Initialize Default joint angle on PR_READY state */
#if 0
            /* Initialization of m_ready_joint_angle on PR_TRANSITION_TO_READY state is obsoleted. */
            {
                _MM_ALIGN16 float target_joint_angle[12];
                _MM_ALIGN16 float pre_joint_angle[12];
#if defined(__INTEL_COMPILER)||defined(__ICC)
                _MM_ALIGN16 Mat3 body_R = Mat3::identity();
#elif defined(__GNUC__)
                const Mat3 body_R = Mat3::Identity();
#endif
                const float foot_l_pitch = 0.0f + LegIKParam::InitialLfoot_pitch;
                const float foot_r_pitch = 0.0f + LegIKParam::InitialRfoot_pitch;
                _MM_ALIGN16 Vec3 body_p = m_pIKMethod->calcik(body_R,
                                                              body_p_default_offset,
                                                              LegIKParam::InitialLfoot_p,
                                                              LegIKParam::InitialRfoot_p,
                                                              foot_l_pitch,
                                                              foot_r_pitch,
                                                              target_joint_angle );
#if ROBOT==0
                for(int i=0;i < 12; i++){
                    m_ready_joint_angle[i] = target_joint_angle[i];
                }
#elif ROBOT==1
                for(int i=0;i < 6; i++){
                    /* m_robot of L1 starts from right leg. */
                    m_ready_joint_angle[i+6] = target_joint_angle[i];
                    m_ready_joint_angle[i]   = target_joint_angle[i+6];
                }
#elif ROBOT==2
                for(int i=0;i < 6; i++){
                    /* m_robot of L1 starts from right leg. */
                    m_ready_joint_angle[i+6] = target_joint_angle[i];
                    m_ready_joint_angle[i]   = target_joint_angle[i+6];
                }
#else
#error "PushRecover setting m_robot->joint(i)-q. Undefined ROBOT Type"
#endif
            }
#endif
        }else{
            transition_interpolator_ratio = 1.0; /* use controller output */
        }
    }

    /* transition */
    if (!is_transition_interpolator_empty){
        /* interpolate joint angle by transition_interpolator_ratio */
        /* ratio=1 ->  controlled joint angle                       */
        /* ratio=0 ->  pass through qRef                            */
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            /* expecting the joint angle under PR_READY state equals default_joint_angle */
            switch (m_current_control_state){
            case PR_TRANSITION_TO_IDLE:
                //m_ref_q[i] = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * m_ready_joint_angle[i];
                m_ref_q[i] = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * m_ref_q_store[i];
                //m_ref_q[i] = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * g_ready_joint_angle[i];
                break;
            case PR_TRANSITION_TO_READY:
                m_ref_q[i] = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * m_ready_joint_angle[i];
                //m_ref_q[i] = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * g_ready_joint_angle[i];
                break;
            default:
                break;
            }
        }
        /* Set Base Frame Reference Position */
        switch (m_current_control_state){
        case PR_TRANSITION_TO_IDLE:
            ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * m_act_world_root_pos;
            break;
        case PR_TRANSITION_TO_READY:
            ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * hrp::Vector3(traj_body_init[0], traj_body_init[1], traj_body_init[2]+LegIKParam::InitialLfoot_p[2]);
            break;
        default:
            break;
        }

        /* Set Base Frame Reference Rotation */
        rats::mid_rot(ref_baseRot, transition_interpolator_ratio, input_baseRot, m_robot->rootLink()->R);

        /* set relative Reference ZMP */
        switch (m_current_control_state){
        case PR_TRANSITION_TO_IDLE:
            //m_rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * hrp::Vector3(0.0f, 0.0f, -Zc);
            m_rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * hrp::Vector3(-default_zmp_offset_l[0], 0.0f, -Zc);
            break;
        case PR_TRANSITION_TO_READY:
            //m_rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * hrp::Vector3(0.0f, 0.0f, -Zc);
            m_rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * hrp::Vector3(-default_zmp_offset_l[0], 0.0f, -Zc);
            break;
        default:
            break;
        }
    }else{
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_ref_q[i] = m_robot->joint(i)->q;
        }
        if(m_current_control_state==PR_IDLE){
            /* Set Base Frame Reference Position */
            ref_basePos = input_basePos;
            /* Set Base Frame Reference Rotation */
            ref_baseRot = input_baseRot;
            /* set relative Reference ZMP */
            m_rel_ref_zmp = input_zmp; /* pass through */
        }else if(m_current_control_state==PR_READY || m_current_control_state==PR_BUSY){
            /* Set Base Frame Reference Position */
            ref_basePos = m_robot->rootLink()->p;
            /* Set Base Frame Reference Rotation */
            ref_baseRot = m_robot->rootLink()->R;
            /* set relative Reference ZMP */
            m_rel_ref_zmp = m_rel_ref_zmp; /* pass through */
        }
    }/* End of Interpolation */

    if(m_wheel_ctrl->isActive()){
#if 0
        const double pitch = m_rpy.data.p;
#else
        double rate = m_rate.data.avy;
        double pitch;
        double accx =  m_acc.data.ax;
        double accy = -m_acc.data.ay;
        double accz = -m_acc.data.az;
        double norm2_acc = accx*accx+accz*accz;
        if( norm2_acc > 25.0 ){
            pitch = atan2(accx, accz);
        }else{
            pitch = 3.14159265/2.0;
        }
        if(loop%250==0){
            std::cout << "[pr] acc = [" << accx << ", " << accy << ", " << accz << "],  pitch = " << pitch << ", norm2_acc = " << norm2_acc << ", rate = " << rate << std::endl;
        }
#endif
#if 0
        m_ref_q[12] = m_robot->joint(12)->q = m_wheel_ctrl->calcOutput(0, act_base_rpy[1], rate, (loop%250==0)?true:false);
        m_ref_q[13] = m_robot->joint(13)->q = m_wheel_ctrl->calcOutput(1, act_base_rpy[1], rate);
#else
        //const double pitch = act_base_rpy[1];
        m_wRef.data[0] = m_wheel_ctrl->calcOutput(0, pitch, rate, (loop%250==0)?true:false);
        m_wRef.data[1] = m_wheel_ctrl->calcOutput(1, pitch, rate);
        m_wheel_brake.data[0] = m_wheel_ctrl->brakeState(0);
        m_wheel_brake.data[1] = m_wheel_ctrl->brakeState(1);
#endif
        if(loop%250==0){
            std::cout << "[pr] m_ref_q[12,13] = ["<<m_ref_q[12]<<", "<<m_ref_q[13]<<"], wref = [" << m_wRef.data[0] << ", " << m_wRef.data[1] << "], act_base_rpy[1] = " << act_base_rpy[1] << ", m_rpy.data.p = " << m_rpy.data.p<<std::endl;
            if(m_wheel_ctrl->brakeState(0)){
                std::cout << "[pr] Braking" << std::endl;
            }else{
                std::cout << "[pr] Brake Free" << std::endl;
            }
        }
    }

    // reference acceleration
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (sen != NULL) {
        hrp::Vector3 imu_sensor_pos = sen->link->p + sen->link->R * sen->localPos;
        hrp::Vector3 imu_sensor_vel = (imu_sensor_pos - prev_imu_sensor_pos)/m_dt;
        // convert to imu sensor local acceleration
#if 0
        //TODO
        //hrp::Vector3 acc = (sen->link->R * sen->localR).transpose() * (imu_sensor_vel - prev_imu_sensor_vel)/m_dt;
#else
        hrp::Vector3 acc = hrp::Vector3::Zero();
#endif
        m_accRef.data.ax = acc(0); m_accRef.data.ay = acc(1); m_accRef.data.az = acc(2);
        m_accRef.tm = m_qRef.tm;
        prev_imu_sensor_pos = imu_sensor_pos;
        prev_imu_sensor_vel = imu_sensor_vel;
    }

    /* set reference force */
    {
        //const std::string legs_ee_name[2]={"lleg","rleg"};
        for (size_t i = 0; i < 2; i++) {
            const int eei = ee_index_lr[i];
            m_ref_force[eei].tm = m_qRef.tm;
            //m_ref_force[eei].data[0] = m_ref_force_vec[eei](0);
            m_ref_force[eei].data[0] = m_ref_force_vec[eei](2);
            m_ref_force[eei].data[1] = 0.0;
            m_ref_force[eei].data[2] = 0.0;
        }
    }

#if 1
    bool error_flag = false;
    for( int i=0; i<m_robot->numJoints(); i++){
        if(i!=12 || i!=13){
            if((m_ref_q[i]>10.0)||(m_ref_q[i]<-10.0)||(std::isnan(m_ref_q[i]))||(!(std::isfinite(m_ref_q[i])))){
                error_flag = true;
                std::cout << "[PR] m_ref_q[" << i << "] =" << m_ref_q[i] << std::endl;
            }
        }
    }
    if(error_flag){
        std::cout << "tmp_state=";
        switch(tmp_state){
        case PR_IDLE:
            std::cout << "PR_IDLE, ";
            break;
        case PR_READY:
            std::cout << "PR_READY, ";
            break;
        case PR_BUSY:
            std::cout << "PR_BUSY, ";
            break;
        case PR_TRANSITION_TO_READY:
            std::cout << "PR_TRANSITION_TO_READY, ";
            break;
        case PR_TRANSITION_TO_IDLE:
            std::cout << "PR_TRANSITION_TO_IDLE, ";
            break;
        default:
            break;
        }
        std::cout << " m_current_control_state=";
        switch(m_current_control_state){
        case PR_IDLE:
            std::cout << "PR_IDLE";
            break;
        case PR_READY:
            std::cout << "PR_READY";
            break;
        case PR_BUSY:
            std::cout << "PR_BUSY";
            break;
        case PR_TRANSITION_TO_READY:
            std::cout << "PR_TRANSITION_TO_READY";
            break;
        case PR_TRANSITION_TO_IDLE:
            std::cout << "PR_TRANSITION_TO_IDLE";
            break;
        default:
            break;
        }
        std::cout << std::endl;
        if(is_transition_interpolator_empty){
            printf("is_transition_interpolator_empty=true\n");
        }else{
            printf("is_transition_interpolator_empty=false\n");
        }
        std::cout << "transition_interpolator_ratio=" << transition_interpolator_ratio << std::endl;
    }
#endif
}; /* end of setTargetDataWithInterpolation */

void PushRecover::setOutputData(const bool shw_msg_flag){
#define DEBUG_PRINT(x)  {if(shw_msg_flag) std::cout << "[pr] " << #x << "=" << x << std::endl;}

    if(shw_msg_flag){
        std::cout << "[pr] "  << __func__ << std::endl;
    }

    /*==================================================*/
    /* Set Target Angle Vector and publish from outport */
    /*==================================================*/
    if(shw_msg_flag) printf("[pr] q=[");
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_qRef.data[i] = m_ref_q[i];
        m_prev_ref_q[i]  = m_ref_q[i];
        if(shw_msg_flag) printf("%+3.1lf",rad2deg(m_ref_q[i]));
        if(shw_msg_flag && (i==m_robot->numJoints()-1)){ printf("]\n");
        }else{
            if(shw_msg_flag) printf(", ");
        }
    }

    // basePos
    m_basePos.data.x = ref_basePos(0);
    m_basePos.data.y = ref_basePos(1);
    m_basePos.data.z = ref_basePos(2);
    m_basePos.tm = m_qRef.tm;
    PRINTVEC3(ref_basePos, shw_msg_flag);
    // baseRpy
    hrp::Vector3 baseRpy = hrp::rpyFromRot(ref_baseRot);
    m_baseRpy.data.r = baseRpy(0);
    m_baseRpy.data.p = baseRpy(1);
    m_baseRpy.data.y = baseRpy(2);
    m_baseRpy.tm = m_qRef.tm;
    PRINTVEC3(baseRpy, shw_msg_flag);

    // basePose
    m_basePose.data.position.x = m_basePos.data.x;
    m_basePose.data.position.y = m_basePos.data.y;
    m_basePose.data.position.z = m_basePos.data.z;
    m_basePose.data.orientation.r = m_baseRpy.data.r;
    m_basePose.data.orientation.p = m_baseRpy.data.p;
    m_basePose.data.orientation.y = m_baseRpy.data.y;
    m_basePose.tm = m_qRef.tm;
    // zmp
    m_zmp.data.x = m_rel_ref_zmp(0);
    m_zmp.data.y = m_rel_ref_zmp(1);
    m_zmp.data.z = m_rel_ref_zmp(2);
    m_zmp.tm = m_qRef.tm;
    m_prev_rel_ref_zmp = m_rel_ref_zmp;
    PRINTVEC3(m_rel_ref_zmp, shw_msg_flag);

    /* Write to OutPort */
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_basePoseOut.write();
    m_qRefOut.write();
    m_zmpOut.write();
    /* TODO */
    m_tauRef.tm = m_qRef.tm;
    // for ( int i = 0; i < m_robot->numJoints(); i++ ){
    //     m_tauRef.data[i] = 0.01*i;
    // }
    m_tauRefOut.write();

    /* TODO */
    if(m_wheel_ctrl->isActive()){
        m_wRef.tm = m_qRef.tm;
        m_wRefOut.write();
        m_wheel_brake.tm = m_qRef.tm;
        m_wheel_brakeOut.write();
    }

    // control parameters
    /* TODO */
    for (unsigned int i=0; i<m_ref_forceOut.size(); i++){
        m_ref_forceOut[i]->write();
    }
    m_accRefOut.write();
    m_contactStatesOut.write();
    m_controlSwingSupportTimeOut.write();
    m_walkingStatesOut.write();
    m_sbpCogOffsetOut.write();
    {
        if(shw_msg_flag){
            if(m_walkingStates.data){
                std::cout << "[pr] walkingstates=true" << std::endl;
            }else{
                std::cout << "[pr] walkingstates=false" << std::endl;
            }
            std::cout << "[pr] controlSwingSupportTime=";
            printf("%lf, %lf\n",m_controlSwingSupportTime.data[0],m_controlSwingSupportTime.data[1]);
            std::cout << "[pr] accRef=";
            printf("%lf, %lf, %lf\n", m_accRef.data.ax, m_accRef.data.ay, m_accRef.data.az);
            std::cout << "[pr] ref_force[0](right)=";
            printf("%lf, %lf, %lf\n",
                   m_ref_force[0].data[0],
                   m_ref_force[0].data[1],
                   m_ref_force[0].data[2]);
            std::cout << "[pr] ref_force[1](left) =";
            printf("%lf, %lf, %lf\n",
                   m_ref_force[1].data[0],
                   m_ref_force[1].data[1],
                   m_ref_force[1].data[2]);
            printf("[pr] sbpCogoft=[");
            printf("%+3.4lf, %+3.4lf, %+3.4lf]\n",m_sbpCogOffset.data.x,m_sbpCogOffset.data.y,m_sbpCogOffset.data.z);
            printf("\n");
        }
    }

    if(shw_msg_flag) printf("\n");

#undef DEBUG_PRINT
};

bool PushRecover::checkEmergencyFlag(void){
    if(emergencyStopReqFlag==true){
        std::cout << "[" << m_profile.instance_name << "] emergency Stop Activated!" << std::endl;
        emergencyStopReqFlag = false;
        return true;
    }
    return false;
};

bool PushRecover::calcWorldForceVector(void){
    //const std::string legs_ee_name[2]={"lleg","rleg"};
    const double contact_decision_threshold_foot = 8.0; // [N]

    for (size_t i = 0; i < 2; i++) {
        //const int eei = ee_index_map[legs_ee_name[i]];
        const int eei = ee_index_lr[i];
        hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(ee_params[eei].fsensor_name);
        hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
        hrp::Matrix33 tmpR;
        rats::rotm3times(tmpR, sensor->link->R, sensor->localR);
        hrp::Vector3 nf = tmpR * hrp::Vector3(m_force[eei].data[0], m_force[eei].data[1], m_force[eei].data[2]);
        hrp::Vector3 nm = tmpR * hrp::Vector3(m_force[eei].data[3], m_force[eei].data[4], m_force[eei].data[5]);

        /* save force on each foot by sensor coords */
        world_sensor_ps[eei] = fsp;
        world_force_ps[eei]  = nf;
        world_force_ms[eei]  = nm;

        // calc ee-local COP
        hrp::Link* target         = m_robot->link(ee_params[eei].ee_target);
        const hrp::Matrix33 eeR   = target->R * ee_params[eei].ee_localR;
        const hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * ee_params[eei].ee_localp)); // ee-local force sensor pos EndEffector座標に変換
        nf = eeR.transpose() * nf;
        nm = eeR.transpose() * nm;
        // ee-local total moment and total force at ee position
        /* センサ値をワールド座標からEndEffector座標に変換*/

        m_prev_act_force_z[eei] = 0.85 * m_prev_act_force_z[eei] + 0.15 * nf(2); // filter, cut off 5[Hz]

        /* Determin contact state on each foot */
        if (m_prev_act_force_z[eei] < contact_decision_threshold_foot) {
            ee_params[eei].act_contact_state = false; /* in the air */
        }else{
            ee_params[eei].act_contact_state = true; /* on ground */
        }
    }
}; /* end of PushRecover::calcWorldForceVector */

/* Copied from Stabilizer.  */
//#define DEBUG_HOGE
PushRecover::PR_CONTACT_STATE PushRecover::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot) const{
  PR_CONTACT_STATE con_state;
  rats::coordinates leg_c[2], tmpc;
  hrp::Vector3 ez = hrp::Vector3::UnitZ();
  hrp::Vector3 ex = hrp::Vector3::UnitX();
  //const std::string legs_ee_name[2]={"lleg","rleg"};
  for (size_t i = 0; i < 2; i++) {
      /* EndEffector pos (足平中心位置) 具体的には (ROBOT).confを参照 */
      const int           eei = ee_index_lr[i];
      hrp::Link*       target = m_robot->link(ee_params[eei].ee_target);
      const hrp::Vector3  eep = target->p + target->R * ee_params[eei].ee_localp;

      leg_c[eei].pos = eep;
      hrp::Vector3 xv1(target->R * ex);
      xv1(2)=0.0;
      xv1.normalize();
      hrp::Vector3 yv1(ez.cross(xv1));
      leg_c[eei].rot(0,0) = xv1(0); leg_c[eei].rot(1,0) = xv1(1); leg_c[eei].rot(2,0) = xv1(2);
      leg_c[eei].rot(0,1) = yv1(0); leg_c[eei].rot(1,1) = yv1(1); leg_c[eei].rot(2,1) = yv1(2);
      leg_c[eei].rot(0,2) = ez(0); leg_c[eei].rot(1,2) = ez(1); leg_c[eei].rot(2,2) = ez(2);
#ifdef DEBUG_HOGE
      if(loop%1000==0){
          if(i==0){
              std::cout << "=========================" << std::endl;
              std::cout << "[pr] calcFootOriginCoords" << std::endl;
              std::cout << "m_robot->rootLink()->p =" << m_robot->rootLink()->p.transpose() << std::endl;
          }
          std::cout << "--target->R=\n" << target->R << std::endl;
          string lr = (i==0)?"Left":"Right";
          std::cout << "--target[" << lr << "]=" << (target->p).transpose() << std::endl;
          std::cout << "--offset[" << lr << "]=" << (target->R * ee_params[eei].ee_localp).transpose() << std::endl;
          std::cout << "--eep[" << lr << "]=" << eep.transpose() << std::endl;
          std::cout << "--rot=\n" << leg_c[eei].rot << std::endl;
          std::cout << "--eei="<<eei<<endl;
      }
#endif
  }
#ifdef DEBUG_HOGE
  if(loop%1000==0){
      std::cout << "ee_params[ee_index_lr[EE_INDEX_RIGHT]].act_contact_state = " << ee_params[ee_index_lr[EE_INDEX_RIGHT]].act_contact_state << std::endl;
      std::cout << "ee_params[ee_index_lr[EE_INDEX_LEFT]].act_contact_state = " << ee_params[ee_index_lr[EE_INDEX_LEFT]].act_contact_state << std::endl;
      std::cout << "ee_params[ee_index_lr[EE_INDEX_RIGHT]].fsensor_name = " << ee_params[ee_index_lr[EE_INDEX_RIGHT]].fsensor_name << std::endl;
      std::cout << "ee_params[ee_index_lr[EE_INDEX_LEFT]].fsensor_name = " << ee_params[ee_index_lr[EE_INDEX_LEFT]].fsensor_name << std::endl;
  }
#endif
  if (ee_params[ee_index_lr[EE_INDEX_RIGHT]].act_contact_state &&
      ee_params[ee_index_lr[EE_INDEX_LEFT]].act_contact_state) {
      /* Both foots are on ground */
      rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
      foot_origin_pos = tmpc.pos;
      foot_origin_rot = tmpc.rot;
      con_state = BOTH_FOOTS;
  } else if (ee_params[ee_index_lr[EE_INDEX_RIGHT]].act_contact_state) {
      /* Right foot is on ground */
      foot_origin_pos = leg_c[ee_index_lr[EE_INDEX_RIGHT]].pos;
      foot_origin_rot = leg_c[ee_index_lr[EE_INDEX_RIGHT]].rot;
      con_state = RFOOT;
  } else if (ee_params[ee_index_lr[EE_INDEX_LEFT]].act_contact_state){
      foot_origin_pos = leg_c[ee_index_lr[EE_INDEX_LEFT]].pos;
      foot_origin_rot = leg_c[ee_index_lr[EE_INDEX_LEFT]].rot;
      con_state = LFOOT;
  } else {
      /* Both foots are on the air */
      rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
      foot_origin_pos = tmpc.pos;
      foot_origin_rot = tmpc.rot;
      con_state = ON_AIR;
  }
#ifdef DEBUG_HOGE
  if(loop%1000==0){
      switch(con_state){
      case BOTH_FOOTS:
          std::cout << "--[\x1b[32mBOTH_FOOT\x1b[39m]" << std::endl;
          break;
      case RFOOT:
          std::cout << "--[\x1b[34mRIGHT_FOOT\x1b[39m]" << std::endl;
          break;
      case LFOOT:
          std::cout << "--[\x1b[33mLEFT_FOOT\x1b[39m]" << std::endl;
          break;
      case ON_AIR:
          std::cout << "--[\x1b[31mON_AIR\x1b[39m]" << std::endl;
          break;
      default:
          std::cout << "--aaaaaaaaaaaaaaaaaa" << std::endl;
          break;
      }
      printf("--foot_pos=[%3.3lf, %3.3lf, %3.3lf]\n",(double)foot_origin_pos[0],(double)foot_origin_pos[1],(double)foot_origin_pos[2] );
      std::cout << "--foot_rot=\n" << foot_origin_rot.transpose() << std::endl;
      std::cout << std::endl;
  }
#endif

  return con_state;
};
#undef DEBUG_HOGE

/* Copied from Stabilizer */
/* ret_zmp is in world coords based on contacting foots */
bool PushRecover::calcZMP(hrp::Vector3& ret_zmp, const double zmp_z) const
{
    const double contact_decision_threshold = 8.0; // [N]
    double tmpzmpx = 0.0;
    double tmpzmpy = 0.0;
    double tmpfz = 0.0, tmpfz2 = 0.0;

    //const std::string legs_ee_name[2]={"lleg","rleg"};

    for (size_t i = 0; i < 2; i++) {
        //const int eei = ee_index_map[legs_ee_name[i]];
        const int eei = ee_index_lr[i];
        const hrp::Vector3 fsp = world_sensor_ps[eei];
        const hrp::Vector3 nf  = world_force_ps[eei];
        const hrp::Vector3 nm  = world_force_ms[eei];
        if(loop%1000==0){
            const hrp::Vector3 fpmm = hrp::Vector3(fsp(0)*1000.0,fsp(1)*1000.0,fsp(2)*1000.0);
            PRINTVEC3(fpmm,true);
            PRINTVEC3(nf,true);
            PRINTVEC3(nm,true);
        }

        tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
        tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
        tmpfz += nf(2); /* calc sum of force z */

        tmpfz2 += m_prev_act_force_z[eei]; /* calc sum of filtered force z in EE coordinates */
    }

    if (tmpfz2 < contact_decision_threshold) {
        ret_zmp = prev_act_zmp;
       return false; // in the air
    } else {
        ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
        return true; // on ground
    }
};

#if 0
hrp::Vector3 PushRecover::calcActCoG_CoGVel(const hrp::Vector3 act_foot_origin_pos,
                                            const hrp::Matrix33 act_foot_origin_rot,
                                            hrp::Vector3& act_cog) const{
    /* calcActCoG */
    act_cog = act_foot_origin_rot.transpose() * (m_robot->calcCM() - act_foot_origin_pos);

    hrp::Vector3 act_cogvel;
    if (m_foots_contact_states != m_prev_foots_contact_states) {
        act_cogvel = (act_foot_origin_rot.transpose() * m_prev_act_foot_origin_rot) * act_cogvel;
    } else {
        act_cogvel = (act_cog - m_prev_act_cog)/m_dt;
    }

    return act_cogvel;
};
#else
hrp::Vector3 PushRecover::calcActCoG_CoGVel(const hrp::Vector3 act_foot_origin_pos,
                                            const hrp::Matrix33 act_foot_origin_rot,
                                            hrp::Vector3& act_cog,
                                            hrp::Vector3& prev_act_cog_base) const{
    hrp::Vector3 act_cogvel, act_cog_base;
    /* calcActCoG_Base */
    act_cog_base = act_foot_origin_rot.transpose() * (m_robot->calcCM() - act_foot_origin_pos);
    if (m_foots_contact_states != m_prev_foots_contact_states) {
        //act_cogvel = (act_foot_origin_rot.transpose() * m_prev_act_foot_origin_rot) * act_cogvel;
        act_cogvel = act_cogvel;
    }else{
        act_cogvel = (act_cog_base - prev_act_cog_base)/m_dt;
    }

    /* lowpass filter */
    //const double g = 0.6838; /* Ts=0.008, dT=0.002, 99%filter */
    const double g = 0.5358; /* Ts=0.012, dT=0.002, 99%filter */
    act_cogvel = m_prev_act_cogvel.array() * (1.0 - g) + act_cogvel.array() * g;

    act_cog = act_cog + act_cogvel * m_dt;
    act_cog[2] = act_cog_base[2]; /* only root height use absolute value from act_foot_origin_pos */
    prev_act_cog_base = act_cog_base;

    return act_cogvel;
};
hrp::Vector3 PushRecover::calcActRootPos(const hrp::Vector3 act_foot_origin_pos, const hrp::Matrix33 act_foot_origin_rot,
                                         hrp::Vector3& act_root_pos,
                                         hrp::Vector3& prev_act_root_base) const{
    hrp::Vector3 act_rootvel, act_root_base;
    /* calcActRoot_Base */
    act_root_base = act_foot_origin_rot.transpose() * (hrp::Vector3::Zero() - act_foot_origin_pos);
    if (m_foots_contact_states != m_prev_foots_contact_states) {
        //act_rootvel = (act_foot_origin_rot.transpose() * m_prev_act_foot_origin_rot) * act_rootvel;
        act_rootvel = act_rootvel;
    }else{
        act_rootvel = (act_root_base - prev_act_root_base)/m_dt;
    }

    /* lowpass filter */
    //const double g = 0.6838; /* Ts=0.008, dT=0.002, 99%filter */
    const double g = 0.5358; /* Ts=0.012, dT=0.002, 99%filter */
    act_rootvel = m_prev_act_rootvel.array() * (1.0 - g) + act_rootvel.array() * g;

    const double act_root_z = act_root_pos[2] * (1.0-g) + act_root_base[2] * g;
    act_root_pos = act_root_pos + act_rootvel * m_dt;
    act_root_pos[2] = act_root_z; /* only root height use absolute value from act_foot_origin_pos */
    prev_act_root_base = act_root_base;

    return act_rootvel;
};
#endif

bool PushRecover::updateToCurrentRobotPose(void){
    // update by current joint angles
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_robot->joint(i)->q = m_qCurrent.data[i];
        m_robot_current->joint(i)->q = m_qCurrent.data[i];
    }
    // tempolary set root link origin.
    m_robot->rootLink()->p = hrp::Vector3::Zero();
    m_robot_current->rootLink()->p = hrp::Vector3::Zero();

    /* calc current robot root Posture */
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    /**
       正しくはrootLink()->R=Identityにした上で
       calcForwardKinematics()をして、
       senR=sen->link->R * sen->localR;
       sen->link->RはIdentityと仮定するとsenR=sen->localR;
     */
    //const hrp::Matrix33 senR = sen->link->R * sen->localR;
    const hrp::Matrix33 senR = sen->localR;
    const hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
#if 0
    m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
#elif 1
    /* m_rpy should be corrected. */
    m_robot->rootLink()->R = act_Rs * senR.transpose();

    const hrp::Matrix33 act_RsRP(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, 0.0));
    m_robot_current->rootLink()->R = act_RsRP * senR.transpose();
#else
    /* m_rpy is already corrected. */
    m_robot->rootLink()->R = act_Rs;
#endif
    m_robot->calcForwardKinematics(); /* FK on actual joint angle */
    m_robot_current->calcForwardKinematics(); /* FK on actual joint angle */
    act_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
#ifdef DEBUG_HOGE
    if(loop%500==0){
        //std::cout << "senR=" << std::endl << senR << std::endl;
        std::cout << "m_rpy=" << (180.0/3.14159265)*m_rpy.data.r << ", " << (180.0/3.14159265)*m_rpy.data.p << ", " << (180.0/3.14159265)*m_rpy.data.y << std::endl;
        hrp::Vector3 tmpv = act_base_rpy.array() * (180.0/3.14159265);
        std::cout << "act_base_rpy=" << tmpv.transpose() << std::endl;
    }
#endif
    // world_force_ps, world_force_ms
    calcWorldForceVector();

    // foot base pos
    hrp::Vector3  act_foot_origin_pos;
    hrp::Matrix33 act_foot_origin_rot;
    m_foots_contact_states = calcFootOriginCoords(act_foot_origin_pos, act_foot_origin_rot);

    {
#ifdef USE_DATALOG
        dlog.act_foot_origin_pos = CONV_HRPVEC3(act_foot_origin_pos);
#endif
    }

    // zmp
    bool on_ground = calcZMP(m_act_zmp, act_foot_origin_pos(2));
    m_rel_act_zmp  = calcRelActZMP(m_act_zmp, m_robot->rootLink()->p, m_robot->rootLink()->R);

    // calculate cog and cogvel. The frame is relative to act_foot_origin_pos.
#if 0
    m_act_cogvel = act_cogvel_filter->passFilter( calcActCoG_CoGVel(act_foot_origin_pos, act_foot_origin_rot, m_act_cog) );
#else
    m_act_cogvel = act_cogvel_filter->passFilter( calcActCoG_CoGVel(act_foot_origin_pos, act_foot_origin_rot, m_act_cog, m_prev_act_cog_base) );
#endif
    // act_root_pos
#if 0
    m_act_root_pos = calcActRootPos(act_foot_origin_pos, act_foot_origin_rot);
#else
    m_act_rootvel = calcActRootPos(act_foot_origin_pos, act_foot_origin_rot, m_act_root_pos, m_prev_act_root_pos_base);
#endif

    /* save current values for next cycle */
    m_prev_act_foot_origin_rot    = act_foot_origin_rot;
    m_prev_act_cog                = m_act_cog;
    m_prev_act_cogvel             = m_act_cogvel;
    m_prev_foots_contact_states   = m_foots_contact_states;
    m_prev_act_root_pos           = m_act_root_pos;
    m_prev_act_rootvel            = m_act_rootvel;

    return on_ground;
}; /* updateToCurrentRobotPose */


bool PushRecover::checkJointVelocity(void){
    bool ret = true;
#if 1
    for(int i=0; i<m_robot->numJoints(); i++){
        const double maxq  = m_robot->joint(i)->ulimit;
        const double minq  = m_robot->joint(i)->llimit;
        const double maxdq = m_robot->joint(i)->uvlimit;
        const double mindq = m_robot->joint(i)->lvlimit;
        if(m_robot->joint(i)->q > maxq){
            m_robot->joint(i)->q = maxq;
            ret = false;
        }
        if(m_robot->joint(i)->q < minq){
            m_robot->joint(i)->q = minq;
            ret = false;
        }
        double dq = (m_robot->joint(i)->q - m_prev_ref_q[i])/m_dt;
        if(dq > maxdq){
            dq = maxdq;
            m_robot->joint(i)->q = m_qCurrent.data[i] + dq*m_dt;
            ret = false;
        }
        if(dq < mindq){
            dq = mindq;
            m_robot->joint(i)->q = m_prev_ref_q[i] + dq*m_dt;
            ret = false;
        }
        m_robot->joint(i)->dq = dq;
    }
#endif
#ifdef DEBUG_HOGE
    if(loop%500==0){
        std::cout << "[pr] " << __func__ << std::endl;
        printf("[pr] m_ref_q=      [");
        for ( int i = 0; i < m_robot->numJoints(); i++ ){
            printf("%+3.1lf",rad2deg(m_ref_q[i]));
            if(i==m_robot->numJoints()-1){
                printf("]");
            }else{
                printf(", ");
            }
        }
        printf("\n[pr] m_robot.q=[");
        for ( int i = 0; i < m_robot->numJoints(); i++ ){
            printf("%+3.1lf",rad2deg(m_robot->joint(i)->q));
            if(i==m_robot->numJoints()-1){
                printf("]\n");
            }else{
                printf(", ");
            }
        }
    }
#endif
    return ret;
};

bool PushRecover::checkBodyPosMergin(const double threshold2, const int loop, const bool mask) const{
    double diff2;
    const double y_margin_gain = 3.0; /* 円領域ではなく、四角形領域で考える */
    /* mm単位での実root_posとref_root_pos誤差の自乗和で判定 */
#if 0
    diff2  = (m_act_root_pos(0) - m_prev_ref_basePos(0))*(m_act_root_pos(0) - m_prev_ref_basePos(0))*(1000.0*1000.0);
    diff2 += (m_act_root_pos(1) - m_prev_ref_basePos(1))*(m_act_root_pos(1) - m_prev_ref_basePos(1))*(1000.0*1000.0);
#elif 0
    diff2  = (m_act_root_pos(0) - (m_prev_ref_basePos(0) + m_prev_rel_ref_zmp(0)))*(m_act_root_pos(0) - (m_prev_rel_ref_zmp(0))) * (1000.0*1000.0);
    diff2 += (m_act_root_pos(1) - (m_prev_ref_basePos(1) + m_prev_rel_ref_zmp(1)))*(m_act_root_pos(1) - (m_prev_rel_ref_zmp(1))) * (1000.0*1000.0);
#elif 0   /* USE Actual ZMP */
    const double diff2x  = (m_rel_act_zmp(0) - m_prev_rel_ref_zmp(0))*(m_rel_act_zmp(0) - m_prev_rel_ref_zmp(0)) * (1000.0*1000.0);
    const double diff2y  = (m_rel_act_zmp(1) - m_prev_rel_ref_zmp(1))*(m_rel_act_zmp(1) - m_prev_rel_ref_zmp(1)) * (1000.0*1000.0);
#elif 1  /* USE Modified body_p */
    const double diff2x  = m_ref_basePos_modif(0) * m_ref_basePos_modif(0) * (1000.0*1000.0);
    const double diff2y  = m_ref_basePos_modif(1) * m_ref_basePos_modif(1) * (1000.0*1000.0);
#else /* 動いていないとき、m_act_root_posはdefault_zmp_offsetだけ動いているはずで、m_rel_ref_zmpは0,0を示すはずだからm_act_root_posを使うのではなく,act_zmpを使うのが正しい? */
    diff2  = (m_act_root_pos(0) - (m_prev_ref_basePos(0) + m_prev_rel_ref_zmp(0)))*(m_act_root_pos(0) - (m_prev_ref_basePos(0) + m_prev_rel_ref_zmp(0))) * (1000.0*1000.0);
    diff2 += (m_act_root_pos(1) - (m_prev_ref_basePos(1) + m_prev_rel_ref_zmp(1)))*(m_act_root_pos(1) - (m_prev_ref_basePos(1) + m_prev_rel_ref_zmp(1))) * (1000.0*1000.0);
#endif

    if(loop%1000==0){
    //if(0){
#if 0
        const float diff_x = (m_act_root_pos(0) - (m_prev_ref_basePos(0) + m_prev_rel_ref_zmp(0)))*1000.0;
        const float diff_y = (m_act_root_pos(1) - (m_prev_ref_basePos(1) + m_prev_rel_ref_zmp(1)))*1000.0;
#elif 0
        const float diff_x = (m_rel_act_zmp(0) - (m_prev_rel_ref_zmp(0)))*1000.0;
        const float diff_y = (m_rel_act_zmp(1) - (m_prev_rel_ref_zmp(1)))*1000.0;
        const float diff_z = (m_rel_act_zmp(2) + Zc) * 1000.0;
#elif 1
        const float diff_x = m_ref_basePos_modif(0) * 1000.0;
        const float diff_y = m_ref_basePos_modif(1) * 1000.0;
        const float diff_z = (m_rel_act_zmp(2) + Zc) * 1000.0;
#else
        const float diff_x = (m_act_root_pos(0) - (m_prev_rel_ref_zmp(0)))*1000.0;
        const float diff_y = (m_act_root_pos(1) - (m_prev_rel_ref_zmp(1)))*1000.0;
        const float diff_z = (m_act_root_pos(2) - Zc) * 1000.0;
#endif
        const double diff  = sqrt(diff2);
#if 0
        std::cout << "[pr] diff=" << diff << "[mm]" << std::endl;
        std::cout << "[pr] diffv=[" << diff_x << ", " << diff_y << ", " << diff_z << "]" << std::endl;
        std::cout << "[pr] foots_contact_state=";
        switch (m_foots_contact_states) {
        case BOTH_FOOTS:
            std::cout << "BOTH_FOOT" << std::endl;
            break;
        case RFOOT:
            std::cout << "RFOOT" << std::endl;
            break;
        case LFOOT:
            std::cout << "LFOOT" << std::endl;
            break;
        case ON_AIR:
            std::cout << "ON AIR" << std::endl;
            break;
        }
        PRINTVEC3(m_act_root_pos, true);
        PRINTVEC3(m_prev_ref_basePos, true);
        PRINTVEC3(m_rel_act_zmp, true);
        PRINTVEC3(m_prev_rel_ref_zmp, true);
        PRINTVEC3(m_act_cogvel, true);
#endif
    }

#if 0 /* Circular Area */
    if(diff2>threshold2 && loop%250==0){
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "=====Invoking PushRecover=====" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    }
    return ((diff2>threshold2)?true:false) & mask;
#else /* Rectanglar Area */
    //if(diff2x>threshold2 && loop%250==0){
    if(diff2x>pushDetectParam.diff_margin_threshold_x && loop%250==0){
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "=====Invoking PushRecover by x=====" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    }
    /* y方向は足幅の分だけ余裕を持たせる */
    //if(diff2y>(threshold2 + 80.0*80.0) && loop%250==0){
    if(diff2y>pushDetectParam.diff_margin_threshold_y && loop%250==0){
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "=====Invoking PushRecover by y=====" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    }
    bool diff_flag;
#if 0
    if(diff2x>threshold2 || diff2y>threshold2*y_margin_gain){
        diff_flag = true;
    }else{
        diff_flag = false;
    }
#else
    if(diff2x>pushDetectParam.diff_margin_threshold_x || diff2y>pushDetectParam.diff_margin_threshold_y){
        diff_flag = true;
    }else{
        diff_flag = false;
    }
#endif

    return diff_flag & mask;
#endif
};

bool PushRecover::controlBodyCompliance(bool is_enable){
    double u;

    const double walking_gain = (m_current_control_state==PR_BUSY)?0.2:1.0;
    const double k[3] = {walking_gain * pushDetectParam.body_compliance_k1, walking_gain * pushDetectParam.body_compliance_k2, walking_gain * pushDetectParam.body_compliance_k3};
    const double maxdd = 0.5*m_dt; /* 0.5m/sec^2 */
    const double maxmodif = 0.1;
#if 1
    if(loop%1000==0){
#if ROBOT==0
        std::cout << "[pr] ROBOT=URATALEG TYPE\n";
#endif
#if ROBOT==1
        std::cout << "[pr] ROBOT=L1 TYPE\n";
#endif
#if ROBOT==2
        std::cout << "[pr] ROBOT=L1W TYPE\n";
#endif
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "controlBodyCompliance()" << MAKE_CHAR_COLOR_DEFAULT << (is_enable?"[ENABLED]":"[DISABLED]") << std::endl;
        std::cout << "[pr] simmode=" << m_simmode << std::endl;
    }
    for(int i = 0; i<2; i++){
        u = k[0] * (m_rel_act_zmp(i) - m_prev_rel_ref_zmp(i)) + k[1] * (m_ref_basePos_modif(i) - 0.0f);
        if(loop%1000==0){
            std::cout << "[pr] zmp_diff=[" << m_rel_act_zmp(i) << " - " << m_prev_rel_ref_zmp(i) << "]" << std::endl;
        }
        double prev_dx = bodyComplianceContext[i].prev_u;
        //act_cogvel;も使う？
        if(u > prev_dx + maxdd){
            u = prev_dx + maxdd;
        }else if(u < prev_dx - maxdd){
            u = prev_dx - maxdd;
        }
        double tmp_x = m_ref_basePos_modif(i) + u;
        if(tmp_x > maxmodif){
            tmp_x = maxmodif;
        }else if(tmp_x < -maxmodif){
            tmp_x = -maxmodif;
        }
        if(is_enable){
            m_ref_basePos_modif(i) = tmp_x;
            bodyComplianceContext[i].prev_u = u;
        }
    }
#endif
#if 1
    if(loop%1000==0){
        std::cout << "[pr] u=[" << bodyComplianceContext[0].prev_u << ", " << bodyComplianceContext[1].prev_u << "]" << std::endl;
        std::cout << "[pr] modif=[" << m_ref_basePos_modif[0] << ", " << m_ref_basePos_modif[1] << "]" << std::endl;
    }
#endif
    /* smoothing by filter */
    //ref_zmp_modif     = ref_zmp_modif_filter->passFilter(ref_zmp_modif);
    ref_zmp_modif     = hrp::Vector3(0.0f, 0.0f, 0.0f);
    m_ref_basePos_modif = ref_basePos_modif_filter->passFilter(m_ref_basePos_modif);

    return true;
}; /* controlBodyCompliance */

void PushRecover::trajectoryReset(void){
    m_abs_est.reset_estimation<true>(Mat3::Identity(), &m_ready_joint_angle[0]);
    m_act_world_root_pos   = hrp::Vector3(traj_body_init[0],
                                          traj_body_init[1],
                                          //traj_body_init[2] + LegIKParam::InitialLfoot_p[2]
                                          traj_body_init[2]
                                          );
    m_body_p_at_start      = m_act_world_root_pos;
    m_body_p_diff_at_start = hrp::Vector3(0.0, 0.0, 0.0);
    m_footl_p_at_start     = hrp::Vector3(0.0, LegIKParam::InitialLfoot_p[1], 0.0);
    m_footr_p_at_start     = hrp::Vector3(0.0, LegIKParam::InitialRfoot_p[1], 0.0);
    m_prev_ref_basePos     = hrp::Vector3(traj_body_init[0],
                                          traj_body_init[1],
                                          traj_body_init[2] + LegIKParam::InitialLfoot_p[2]
                                          );

    m_rel_ref_zmp       = hrp::Vector3(-default_zmp_offset_l[0], 0.0f, -m_prev_ref_basePos[2]);
    m_prev_rel_ref_zmp  = m_rel_ref_zmp;

    for(int i=0;i<2;i++){
        bodyComplianceContext[i].prev_u = 0.0;
    }
    m_ref_basePos_modif = hrp::Vector3::Zero();
    ref_basePos_modif_filter->reset(m_ref_basePos_modif);
    ref_zmp_modif     = hrp::Vector3::Zero();
    m_basePos_modif_at_start = hrp::Vector3::Zero();

    m_prev_ref_traj.clear();

    {
        const int inc_frame = (int)(m_dt*1000);
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
        while( m_owpg.isComplete() != 1){
            m_owpg.incrementFrameNoIdle2<LegIKParam,LEG_IK_TYPE>(inc_frame, m_owpg_state, m_owpg_cstate, false);
        }
        m_prev_owpg_isComplete = false;
        m_owpg_state.body_p = Vec3::Zero();
        m_owpg_state.p      = Vec3::Zero();
        m_owpg_state.foot_l_p = Vec3::Zero();
        m_owpg_state.foot_r_p = Vec3::Zero();
        InitContext(&ctx);
        InitContext(&cty);

        m_owpg.resetFilter();
        m_modify_rot_context.copyWalkParam( &m_owpg );
        m_modify_rot_context.reset();
#else
        while( m_owpg.isComplete() != 1){
            const bool use_iterate = false;
            m_owpg.incrementFrame<LegIKParam,LEG_IK_TYPE,use_iterate>(inc_frame, m_owpg_state, false);
        }
        m_prev_owpg_isComplete = false;
        m_owpg_state.body_p = Vec3::Zero();
        m_owpg_state.p      = Vec3::Zero();
        m_owpg_state.foot_l_p = Vec3::Zero();
        m_owpg_state.foot_r_p = Vec3::Zero();

        m_owpg.resetFilter();
        m_modify_rot_context.copyWalkParam( &m_owpg );
        m_modify_rot_context.reset();
#endif

        m_joystate.reset();
    }

    m_act_cog      = hrp::Vector3::Zero();
    m_act_cogvel   = hrp::Vector3::Zero();
    m_act_root_pos = hrp::Vector3::Zero();
    m_act_rootvel  = hrp::Vector3::Zero();
    m_prev_act_cog      = hrp::Vector3::Zero();
    m_prev_act_cog_base = hrp::Vector3::Zero();
    m_prev_act_root_pos_base = hrp::Vector3::Zero();
    m_prev_act_cogvel   = hrp::Vector3::Zero();
    m_prev_act_rootvel  = hrp::Vector3::Zero();
    m_prev_ref_cog      = hrp::Vector3::Zero();
    m_ref_cogvel   = hrp::Vector3::Zero();
}; /* TrajectoryReset() */

// void PushRecover::convertRefForceToTorque(void){
    
// }; /* convertRefForceToTorque */

RTC::ReturnCode_t PushRecover::onExecute(RTC::UniqueId ec_id)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);

  if (loop%10000==0){
      std::cout << "[" << m_profile.instance_name<< "] onExecute(" << ec_id << ")" << std::endl;
      std::cout << "[" << m_profile.instance_name<< "] ROBOT = " << RobotConfiguration::robotname << std::endl;
      /* Show current state */
      std::string state_string;
      switch(m_current_control_state){
      case PR_IDLE:
          state_string = "PR_IDLE";
          break;
      case PR_READY:
          state_string = "PR_READY";
          break;
      case PR_BUSY:
          state_string = "PR_BUSY";
          break;
      default:
          state_string = "Under Transition";
          break;
      }
      std::cout << "[" << m_profile.instance_name << "] Current State=\"" << state_string.c_str() << "\"" << std::endl;
      bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
      if(is_transition_interpolator_empty){
          std::cout << "[" << m_profile.instance_name << "] interpolator empty" << std::endl;
      }else{
          std::cout << "[" << m_profile.instance_name << "] interpolator working" << std::endl;
      }
      std::cout << "[" << m_profile.instance_name << "] m_robot->numJoints() = " << m_robot->numJoints() << std::endl;
  }

  /* check dataport input */
  {
      const bool shw_msg_flag = loop%500==0?true:false;
      //updateInputData(shw_msg_flag);
      //updateInputData(true);
      updateInputData(false);
      //updateRotContext( Vec3(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y ), m_modify_rot_context );
      updateRotContext( Vec3(act_base_rpy[0], act_base_rpy[1], act_base_rpy[2]), m_modify_rot_context );
  }
  /* End of checking dataport input */

  {/* Mutex lock region */
      Guard guard(m_mutex);
#ifdef USE_DATALOG
      /* zero clear dlog */
      memset(&dlog,0,sizeof(SimpleLogger::DataLog));
#endif

      /* calculate actual zmp, cog, cogvel, poture from joint_angle, force, rpy */
      const bool on_ground = (updateToCurrentRobotPose() || m_simmode>0);

      /* calculate estimated cogvel from imu rate */
      /* TODO use est_cogvel */
      m_est_cogvel_from_rpy = updateEstimatedInputData();

#if 1
      {
          Vec3 imu_rpy, body_p;
          _MM_ALIGN32 float q[12];
          imu_rpy = Vec3(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y);
          body_p = Vec3::Zero();
#if ROBOT==0
          for ( std::size_t i = 0; i < m_robot->numJoints(); i++ ){
              q[i] = m_robot->joint(i)->q;
          }
#elif ROBOT==1
          for ( std::size_t i = 0; i < 6; i++ ){
              q[i]   = m_robot->joint(i+6)->q;
              q[i+6] = m_robot->joint(i)->q;
          }
#elif ROBOT==2
          for ( std::size_t i = 0; i < 6; i++ ){
              q[i]   = m_robot->joint(i+6)->q;
              q[i+6] = m_robot->joint(i)->q;
          }
#else
#error "PushRecover Undefined ROBOT Type"
#endif

#if 1
          _MM_ALIGN16 float fl[6], fr[6];
          for(std::size_t i=0; i<6; i++){
              fl[i] = m_force[ee_index_lr[EE_INDEX_LEFT]].data[i];
              fr[i] = m_force[ee_index_lr[EE_INDEX_RIGHT]].data[i];
          }
#else
          /* for Simulation only */
          float fl[6];
          float fr[6];
          std::fill(fl, &fl[6], 0.0f);
          std::fill(fr, &fr[6], 0.0f);
          if(m_owpg.isComplete()){
              fl[2] = -250.0f;
              fr[2] = -250.0f;
          }else{
              if(m_owpg.isSwingLeg(1)){
                  fl[2] = 0.0f;
              }else{
                  fl[2] = -250.0f;
              }
              if(m_owpg.isSwingLeg(-1)){
                  fr[2] = 0.0f;
              }else{
                  fr[2] = -250.0f;
              }
          }
#endif
#if defined(USE_APECF)
          m_abs_est.setAccelerationSensorVector(m_acc.data.ax, m_acc.data.ay, m_acc.data.az);
#endif
          m_abs_est.setGyroRateVector(m_rate.data.avx, m_rate.data.avy, m_rate.data.avz);
          m_abs_est.setForceSensorVector(fl, fr);
          m_abs_est.updateToCurrentRobotPose(m_dt, imu_rpy, q);
          Vec3 abs_zmp = *(m_abs_est.getZMP());
          Vec3 abs_rel_act_zmp = *(m_abs_est.getRelActZMP());
          Vec3 abs_contact_state = (*(m_abs_est.getContactState()));
          Vec3 abs_foot_origin_pos = *(m_abs_est.getFootOriginPos());
          Mat3 abs_foot_origin_rot = *(m_abs_est.getFootOriginRot());
          Vec3 abs_body_p = *(m_abs_est.getAbsBodyPos());
          Vec3 abs_body_v = *(m_abs_est.getAbsBodyVel());
          Vec3 abs_cog_p = *(m_abs_est.getAbsCoGPos());
          Vec3 abs_cog_v = *(m_abs_est.getAbsCoGVel());
          const Vec3 abs_foot_origin_rpy = bodylink::rot2rpy<bodylink::ROBOTICS>(abs_foot_origin_rot);
          const Vec3 rel_act_cp = *(m_abs_est.getRelActCP());
          dlog.abs_zmp = CONV_VEC3( abs_zmp );
          dlog.abs_rel_act_zmp = CONV_VEC3( abs_rel_act_zmp );
          dlog.abs_contact_state = CONV_VEC3( abs_contact_state );
          dlog.abs_foot_origin_pos = CONV_VEC3( abs_foot_origin_pos );
          dlog.abs_foot_origin_rpy = CONV_VEC3( abs_foot_origin_rpy );
          dlog.abs_body_p = CONV_VEC3(abs_body_p);
          dlog.abs_body_v = CONV_VEC3(abs_body_v);
          dlog.abs_cog_p = CONV_VEC3(abs_cog_p);
          dlog.abs_cog_v = CONV_VEC3(abs_cog_v);
          dlog.rel_act_cp = CONV_VEC3(rel_act_cp);

          {
              const float cp_x_thre = 0.15f;
              const float cp_y_thre = 0.15f;
              Vec3 rel_cp = *(m_abs_est.getRelActCP());
              if((((cp_x_thre < rel_cp[0]) || (-cp_x_thre > rel_cp[0])) || ((cp_y_thre < rel_cp[1]) || (-cp_y_thre > rel_cp[1])))&&loop%500==0){
                  std::cout << "[pr] " << PRED << "CP Detect Fall down!!!!" << PDEF << "[" << rel_cp[0] << ", " << rel_cp[1] << "]" << std::endl;
              }else if(loop%500==0){
                  //std::cout << "[pr] " << " CP [" << rel_cp[0] << ", " << rel_cp[1] << "]" << std::endl;
              }
          }

          PoseState pose_state;
          m_abs_est.getAbsolutePoseState(pose_state);

          //if( loop%500==0 ){
          if( 0 ){
              std::cout << "[pr] zmp="<< abs_zmp.transpose() << ", contact=" << abs_contact_state.transpose() << std::endl;
              Vec3 fl,ml,fr,mr;
              m_abs_est.getWorldForce(fl,ml,fr,mr);
              //std::cout << "[pr] fl="<<fl.transpose() << ", ml=" << ml.transpose() << ", fr="<<fr.transpose() << ", mr=" << mr.transpose() << std::endl;
              m_abs_est.getWorldEEForce(fl,ml,fr,mr);
              //std::cout << "[pr] fl="<<fl.transpose() << ", ml=" << ml.transpose() << ", fr="<<fr.transpose() << ", mr=" << mr.transpose() << std::endl;
              std::cout << "[pr] rpy=[" << m_rpy.data.r << ", " << m_rpy.data.p << ", " << m_rpy.data.y << "]\n";
              //std::cout << "[pr] rate=[" << m_rate.data.avx << ", " << m_rate.data.avy << ", " << m_rate.data.avz << "]\n";
              //std::cout << "[pr] acc =[" << m_acc.data.ax << ", " << m_acc.data.ay << ", " << m_acc.data.az << "]\n";
              std::cout << "[pr] abs_body_p=[" << abs_body_p.transpose() << "]\n";
              std::cout << "[pr] abs_body_v=[" << abs_body_v.transpose() << "]\n";
              //std::cout << "[pr] abs_cog_p=[" << abs_cog_p.transpose() << "]\n";
              //std::cout << "[pr] abs_cog_v=[" << abs_cog_v.transpose() << "]\n";
              std::cout << "[pr] body_p_at_start="<<m_body_p_at_start.transpose() << "\n";
              std::cout << "[pr] footl_p_at_start="<<m_footl_p_at_start.transpose() << "\n";
              std::cout << "[pr] footr_p_at_start="<<m_footr_p_at_start.transpose() << "\n";
#if defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
              std::cout << "[pr] x_offset="<<(m_owpg.getXOffset()).transpose() << std::endl;
#endif
              std::cout << "[pr] zmp="<<(pose_state.zmp).transpose() << std::endl;
              std::cout << "[pr] body_p="<<pose_state.body.p.transpose() << std::endl;
              std::cout << "[pr] body_rpy="<< (bodylink::rot2rpy<bodylink::ROBOTICS>(pose_state.body.R)).transpose() << std::endl;
              std::cout << "[pr] footl_p="<<pose_state.footl.p.transpose() << std::endl;
              std::cout << "[pr] footl_rpy="<< (bodylink::rot2rpy<bodylink::ROBOTICS>(pose_state.footl.R)).transpose() << std::endl;
              std::cout << "[pr] footr_p="<<pose_state.footr.p.transpose() << std::endl;
              std::cout << "[pr] footr_rpy="<< (bodylink::rot2rpy<bodylink::ROBOTICS>(pose_state.footr.R)).transpose() << std::endl;
#if 0
              std::cout << "[pr] ustate.body_p =" << (-Vec3(m_body_p_at_start[0], m_body_p_at_start[1], 0.0f)+ Vec3(pose_state.body.p[0], pose_state.body.p[1], 0.0f)).transpose() << std::endl;
              std::cout << "[pr] ustate.foot_l_p =" << (-Vec3(m_footl_p_at_start[0],m_footl_p_at_start[1]-LegIKParam::InitialLfoot_p[1],m_footl_p_at_start[2]) + pose_state.footl.p).transpose() << std::endl;
              std::cout << "[pr] ustate.foot_r_p =" << (-Vec3(m_footr_p_at_start[0],m_footr_p_at_start[1]-LegIKParam::InitialRfoot_p[1],m_footr_p_at_start[2]) + pose_state.footr.p).transpose() << std::endl;
#else
              std::cout << "[pr] ustate.body_p =" << (Vec3(pose_state.body.p[0]-traj_body_init[0], pose_state.body.p[1], 0.0f)).transpose() << std::endl;
              std::cout << "[pr] ustate.foot_l_p =" << (pose_state.footl.p).transpose() << std::endl;
              std::cout << "[pr] ustate.foot_r_p =" << (pose_state.footr.p).transpose() << std::endl;
#endif
          }
      }
#endif
#ifdef USE_DATALOG
      {
          dlog.act_zmp        = CONV_HRPVEC3(m_act_zmp);
          dlog.rel_act_zmp    = CONV_HRPVEC3(m_rel_act_zmp);
          dlog.act_cog        = CONV_HRPVEC3(m_act_cog);
          dlog.act_root_pos   = CONV_HRPVEC3(m_act_root_pos);
          dlog.act_contact_state[0] = (float)((uint32_t)ee_params[ee_index_lr[EE_INDEX_LEFT]].act_contact_state);
          dlog.act_contact_state[1] = (float)((uint32_t)ee_params[ee_index_lr[EE_INDEX_RIGHT]].act_contact_state);
      }
#endif

      // TODO set modified m_ref_basePos_modif and ref_zmp_modif
      {
#if 0
          const bool do_body_compliance = (((m_current_control_state==PR_READY)||(m_current_control_state==PR_BUSY))&&(m_simmode==0))?true:false;
#else
          const bool do_body_compliance = false;
#endif
          //controlBodyCompliance(do_body_compliance);
      }

      {
#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
          interpretJoystickCommandandSend(m_joyaxes, m_joybuttons, m_joystate, &m_owpg);
#else
          interpretJoystickCommandandSend(m_joyaxes, m_joybuttons, m_joystate, &m_owpg);
#endif
      }

      if(m_current_control_state==PR_READY || m_current_control_state==PR_BUSY){
          if(m_simmode>0){
              if(DEBUGP(1000)){
                  std::cout << "[pr] SIMMODE WARNING!!\nbasepos_modif and act_cogvel is force replaced by zero.\n";
              }
              /* Force modify m_ref_basePos_modif and m_act_cogvel according to the m_simmode */
              selectSimmodeControlValue(m_simmode, m_ref_basePos_modif, m_act_cogvel);
          }/* End of simmode setting dummy basePos modification and velocity */

          TrajectoryElement<Vec3e> ref_traj;

          const bool suppress_pr = !( !m_walkingStates.data && on_ground && m_current_control_state != PR_BUSY);
          const int pattern_generator_selector = m_generator_select; // latch generator_select value
          const bool enable_modify = true;
          switch(pattern_generator_selector){
          case 0:
              /* PushRecover */
              executeActiveStateExtractTrajectory(suppress_pr,
                                                  on_ground,
                                                  m_ref_basePos_modif,
                                                  ref_traj);
              break;
          case 1:
              /* OnlineWalkingPatternGenerator */
              executeActiveStateExtractTrajectoryOnline(on_ground,
                                                        m_ref_basePos_modif,
                                                        ref_traj);
              // modifyTrajectoryRot(enable_modify, on_ground,
              //                     m_modify_rot_context,
              //                     ref_traj);
              //modifyFootHeight(on_ground, m_modify_rot_context, ref_traj);
              break;
          case 2:
              break;
          default:
              ref_traj = m_prev_ref_traj;
              break;
          }
          /* Finally set rootlink pos to world */
          updateRefPos_ZMP( ref_traj, m_ref_basePos_modif, m_act_world_root_pos );
          //m_robot->rootLink()->p = m_act_world_root_pos + ref_basePos_modif;
          m_robot->rootLink()->p = m_act_world_root_pos;

          /* Save log */
          {
#ifdef USE_DATALOG
              dlog.sf_pref          = CONV_VEC3(ref_traj.p);
              dlog.sf_body_p        = CONV_VEC3(ref_traj.body_p);
              dlog.sf_footl_p       = CONV_VEC3(ref_traj.footl_p);
              dlog.sf_footr_p       = CONV_VEC3(ref_traj.footr_p);
              dlog.ref_traj_dp      = CONV_VEC3(ref_traj.dp);
              dlog.ref_traj_body_dp = CONV_VEC3(ref_traj.body_dp);
              dlog.filtered_rot  = CONV_VEC3( m_modify_rot_context.filtered_rot );
              dlog.lpf_rot  = CONV_VEC3( m_modify_rot_context.lpf_rot );
              dlog.rot_offset  = CONV_VEC3( m_modify_rot_context.rot_offset );
#endif
          }
          executeActiveStateCalcJointAngle(ref_traj, m_ref_basePos_modif);

          {/* set current walking status */
              /* TODO */
#if 1
              m_contactStates.data[ee_index_lr[EE_INDEX_LEFT]] = (ref_traj.footl_p[2]<1e-3)?true:false;
              m_contactStates.data[ee_index_lr[EE_INDEX_RIGHT]] = (ref_traj.footr_p[2]<1e-3)?true:false;
              if(m_generator_select==1){
                  double time[2];
                  extractSwingSupportTime(time);
                  m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_LEFT]]  = time[0];
                  m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_RIGHT]] = time[1];
              }else{
                  m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_LEFT]]  = m_contactStates.data[ee_index_lr[EE_INDEX_LEFT]]?1.0:0.0;
                  m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_RIGHT]] = m_contactStates.data[ee_index_lr[EE_INDEX_RIGHT]]?1.0:0.0;
              }

#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
              m_walkingStates.data = !m_owpg.isComplete();
#else
              m_walkingStates.data = !m_owpg.isComplete();
#endif
#else
              if(m_current_control_state == PR_BUSY){
                  if(DEBUGP(1000)){
                      std::cout << "[PR_BUSY] set current walkking status" << std::endl;
                  }
                  m_contactStates.data[ee_index_lr[EE_INDEX_RIGHT]] = (abs(ref_traj.footr_p[2])<1e-3)?true:false;
                  m_contactStates.data[ee_index_lr[EE_INDEX_LEFT]] = (abs(ref_traj.footl_p[2])<1e-3)?true:false;
                  m_walkingStates.data = true;
                  /* controlSwingSupportTime is not used while double support period, 1.0 is neglected */
                  //m_controlSwingSupportTime.data[ee_index_map["rleg"]] = 1.0;
                  //m_controlSwingSupportTime.data[ee_index_map["lleg"]] = 1.0;
              }else if(m_current_control_state == PR_READY){
                  if(DEBUGP(1000)){
                      std::cout << "[PR_READY] set current walkking status" << std::endl;
                  }
                  /* set current walking status */
                  m_contactStates.data[ee_index_lr[EE_INDEX_RIGHT]] = true;
                  m_contactStates.data[ee_index_lr[EE_INDEX_LEFT]] = true;
                  m_walkingStates.data = false;
                  /* controlSwingSupportTime is not used while double support period, 1.0 is neglected */
                  m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_RIGHT]] = 1.0;
                  m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_LEFT]] = 1.0;
              }
#endif
#if 0
              { /* set reference force */
#if 0
                  const hrp::Vector3 footl_p_ee =
                      hrp::Vector3(ref_traj.footl_p[0]+LegIKParam::InitialLfoot_p[0],
                                   ref_traj.footl_p[1]+LegIKParam::InitialLfoot_p[1],
                                   ref_traj.footl_p[2]);
                  const hrp::Vector3 footr_p_ee =
                      hrp::Vector3(ref_traj.footr_p[0]+LegIKParam::InitialRfoot_p[0],
                                   ref_traj.footr_p[1]+LegIKParam::InitialRfoot_p[1],
                                   ref_traj.footr_p[2]);
#else /* こっちが正解かな */
                  const hrp::Vector3 footl_p_ee =
                      hrp::Vector3(ref_traj.footl_p[0],
                                   ref_traj.footl_p[1],
                                   ref_traj.footl_p[2]);
                  const hrp::Vector3 footr_p_ee =
                      hrp::Vector3(ref_traj.footr_p[0],
                                   ref_traj.footr_p[1],
                                   ref_traj.footr_p[2]);
#endif
                  double alpha = (m_ref_zmp - footr_p_ee).norm() / (footr_p_ee - footl_p_ee).norm();
                  if (alpha>1.0) alpha = 1.0;
                  if (alpha<0.0) alpha = 0.0;
                  m_ref_force_vec[0](0) = alpha * m_mg;     /*ref_force right*/
                  m_ref_force_vec[1](0) = (1-alpha) * m_mg; /*ref_force left*/
              }
#elif 0
              if( m_contactStates.data[ee_index_lr[EE_INDEX_RIGHT]] && m_contactStates.data[ee_index_lr[EE_INDEX_LEFT]] ) {
                  m_ref_force_vec[0](0) = m_mg2;
                  m_ref_force_vec[1](0) = m_mg2;
              }else{
                  m_ref_force_vec[0](0) = m_contactStates.data[ee_index_lr[EE_INDEX_RIGHT]]?(m_mg):0.0;     /*ref_force right*/
                  m_ref_force_vec[1](0) = m_contactStates.data[ee_index_lr[EE_INDEX_LEFT]]?(m_mg):0.0;     /*ref_force left */
              }
#elif 1
              /* m_robot_current->rootLink()->RはIMUのRollとPitchのみ考慮 */
              m_ref_force_vec[EE_INDEX_LEFT] = m_robot_current->rootLink()->R.transpose() * hrp::Vector3(m_mg*ref_traj.footl_f[0],
                                                                                             m_mg*ref_traj.footl_f[1],
                                                                                             m_mg*ref_traj.footl_f[2]);
              m_ref_force_vec[EE_INDEX_RIGHT] = m_robot_current->rootLink()->R.transpose() * hrp::Vector3(m_mg*ref_traj.footr_f[0],
                                                                                              m_mg*ref_traj.footr_f[1],
                                                                                              m_mg*ref_traj.footr_f[2]);
#else
              m_ref_force_vec[0](0) = 0.0;
              m_ref_force_vec[1](0) = 0.0;
#endif
          }

          if( pattern_generator_selector == 0 ){
              const int rwg_len_out = 4500; /* TODO */
              if(rate_matcher.getConvertedFrame()>rwg_len_out){
                  stpf.reset();
                  m_current_control_state = PR_READY;
              }
          }
      } else if(m_current_control_state == PR_IDLE){
          for ( int i = 0; i < m_robot->numJoints(); i++ ) {
              m_robot->joint(i)->q = m_qRef.data[i]; /* pass through qRefIn joint angle */
          }
          m_robot->rootLink()->p = input_basePos;
          m_robot->rootLink()->R = input_baseRot;

          //m_ref_force_vec[0](0) = m_ref_force[0].data[0]; /*ref_force right*/
          //m_ref_force_vec[1](0) = m_ref_force[1].data[0]; /*ref_force left*/
          m_ref_force_vec[EE_INDEX_LEFT] = hrp::Vector3::Zero();
          m_ref_force_vec[EE_INDEX_RIGHT] = hrp::Vector3::Zero();
      } else { /* Transition state */
          for ( int i = 0; i < m_robot->numJoints(); i++ ) {
              //m_robot->joint(i)->q = m_qCurrent.data[i];
              m_robot->joint(i)->q = m_prev_ref_q[i];
          }
          m_robot->rootLink()->p = input_basePos;
          m_robot->rootLink()->R = input_baseRot;
      }

      m_robot->calcForwardKinematics(); /* FK on target joint angle */
      m_prev_ref_cog = m_ref_cog;
      m_ref_cog = m_robot->calcCM();
      m_ref_cogvel = (m_ref_cog - m_prev_ref_cog)*m_dt_i;

      if(( m_current_control_state == PR_READY ) || ( m_current_control_state == PR_BUSY )){
          if( !checkJointVelocity() ){
              //std::cout << "[pr] checkJointVelocity [" << MAKE_CHAR_COLOR_RED << "Failed" << MAKE_CHAR_COLOR_DEFAULT << "]" << std::endl;
          }
      }
#if 0
      printMembers(100);
#endif

      /*==================================================*/
      /* Interpolate Target angle and qRefIn angle        */
      /*==================================================*/
      setTargetDataWithInterpolation();

  }/* Mutex locked region */

  /* calc torque */
  {
#if ROBOT==0
      //#error "m_tauRef calculation is not defined."
      /* Donot use torque mode for L0. */
      for(int j=0; j<6; j++){
          m_tauRef.data[j+6] = 0.0f;
      }
#elif ROBOT==1
      { //calc Left
          hrp::dmatrix ee_J = hrp::dmatrix::Zero(6, 6);
          m_pleg[EE_INDEX_LEFT]->calcJacobian(ee_J);
          Eigen::VectorXd ee_f = Eigen::VectorXd::Zero(6);
          ee_f[0] = m_ref_force_vec[EE_INDEX_LEFT][0];
          ee_f[1] = m_ref_force_vec[EE_INDEX_LEFT][1];
          ee_f[2] = m_ref_force_vec[EE_INDEX_LEFT][2];
          ee_f[3] = 0.0;
          ee_f[4] = 0.0;
          ee_f[5] = 0.0;
          const Eigen::VectorXd tauref = ee_J.transpose() * ee_f;
          for(int j=0; j<6; j++){
              m_tauRef.data[j+6] = tauref[j];
          }
      }
      { //calc Right
          hrp::dmatrix ee_J = hrp::dmatrix::Zero(6, 6);
          m_pleg[EE_INDEX_RIGHT]->calcJacobian(ee_J);
          Eigen::VectorXd ee_f = Eigen::VectorXd::Zero(6);
          ee_f[0] = m_ref_force_vec[EE_INDEX_RIGHT][0];
          ee_f[1] = m_ref_force_vec[EE_INDEX_RIGHT][1];
          ee_f[2] = m_ref_force_vec[EE_INDEX_RIGHT][2];
          ee_f[3] = 0.0;
          ee_f[4] = 0.0;
          ee_f[5] = 0.0;
          const Eigen::VectorXd tauref = ee_J.transpose() * ee_f;
          for(int j=0; j<6; j++){
              m_tauRef.data[j] = tauref[j];
          }
      }
#elif ROBOT==2
      { //calc Left
          hrp::dmatrix ee_J = hrp::dmatrix::Zero(6, 6);
          m_pleg[EE_INDEX_LEFT]->calcJacobian(ee_J);
          Eigen::VectorXd ee_f = Eigen::VectorXd::Zero(6);
          ee_f[0] = m_ref_force_vec[EE_INDEX_LEFT][0];
          ee_f[1] = m_ref_force_vec[EE_INDEX_LEFT][1];
          ee_f[2] = m_ref_force_vec[EE_INDEX_LEFT][2];
          ee_f[3] = 0.0;
          ee_f[4] = 0.0;
          ee_f[5] = 0.0;
          const Eigen::VectorXd tauref = ee_J.transpose() * ee_f;
          for(int j=0; j<6; j++){
              m_tauRef.data[j+6] = tauref[j];
          }
      }
      { //calc Right
          hrp::dmatrix ee_J = hrp::dmatrix::Zero(6, 6);
          m_pleg[EE_INDEX_RIGHT]->calcJacobian(ee_J);
          Eigen::VectorXd ee_f = Eigen::VectorXd::Zero(6);
          ee_f[0] = m_ref_force_vec[EE_INDEX_RIGHT][0];
          ee_f[1] = m_ref_force_vec[EE_INDEX_RIGHT][1];
          ee_f[2] = m_ref_force_vec[EE_INDEX_RIGHT][2];
          ee_f[3] = 0.0;
          ee_f[4] = 0.0;
          ee_f[5] = 0.0;
          const Eigen::VectorXd tauref = ee_J.transpose() * ee_f;
          for(int j=0; j<6; j++){
              m_tauRef.data[j] = tauref[j];
          }
      }
#else
#error "PushRecover::onexecute() Definition of ROBOT is unavailable."
#endif
#if 0
      if(loop%500==0){
          std::cout << "[pr] m_tauRef=[";
          for(int i=0;i<m_robot->numJoints(); i++){
              printf("%5.3lf",m_tauRef.data[i]);
              if(i==m_robot->numJoints()-1){
                  std::cout << "]\n";
              }else{
                  std::cout << ", ";
              }
          }
      }
#endif
  }/* calc torque*/

  /* Save log */
  if(true){
#ifdef USE_DATALOG
      dlog.sectime   = ((float)(tv.tv_sec-stv.tv_sec)) + ((float)(tv.tv_usec - stv.tv_usec)/1000000.0f);
      //dlog.gclk       = ((((float)(tv.tv_sec-stv.tv_sec)) + ((float)(tv.tv_usec - stv.tv_usec)/1000000.0f)) * 1e8);
      dlog.frame      = (float)rate_matcher.getCurrentFrame();
      dlog.loop       = (float)loop;
      for(int i=0;i<12;i++){
          dlog.act_q[i]  = (float)rad2deg(m_qCurrent.data[i]);
          dlog.ref_q[i]  = (float)rad2deg(m_ref_q[i]);
          dlog.ref_dq[i] = (float)rad2deg(m_robot->joint(i)->dq);
      }
      dlog.rpy  = dlog::V3((float)m_rpy.data.r, (float)m_rpy.data.p, (float)m_rpy.data.y);
      dlog.ref_zmp       = CONV_HRPVEC3(m_ref_zmp);
      dlog.rel_ref_zmp   = CONV_HRPVEC3(m_rel_ref_zmp);
      dlog.ref_base_pos  = CONV_HRPVEC3(ref_basePos);
      dlog.ref_cog       = CONV_HRPVEC3(m_ref_cog);
      for(int i=0;i<6;i++){
          dlog.act_force_l[i] = (float)m_force[ee_index_lr[EE_INDEX_LEFT]].data[i];
          dlog.act_force_r[i] = (float)m_force[ee_index_lr[EE_INDEX_RIGHT]].data[i];
          dlog.ref_force_l[i] = (float)m_ref_force[ee_index_lr[EE_INDEX_LEFT]].data[i];
          dlog.ref_force_r[i] = (float)m_ref_force[ee_index_lr[EE_INDEX_RIGHT]].data[i];
      }
      dlog.contact_state[0] = (float)((int)m_contactStates.data[ee_index_lr[EE_INDEX_LEFT]]);
      dlog.contact_state[1] = (float)((int)m_contactStates.data[ee_index_lr[EE_INDEX_RIGHT]]);
      dlog.walking_state    = (float)((int)m_walkingStates.data);
      dlog.controlSwingSupportTime[0] = (float)m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_LEFT]];
      dlog.controlSwingSupportTime[1] = (float)m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_RIGHT]];
      dlog.sbpCogOffset[0] = m_sbpCogOffset.data.x;
      dlog.sbpCogOffset[1] = m_sbpCogOffset.data.y;
      dlog.sbpCogOffset[2] = m_sbpCogOffset.data.z;

      dlog.act_cogvel         = CONV_HRPVEC3(m_act_cogvel);
      dlog.ref_zmp_modif      = CONV_HRPVEC3(ref_zmp_modif);
      dlog.ref_basePos_modif  = CONV_HRPVEC3(m_ref_basePos_modif);
      dlog.act_world_root_pos = CONV_HRPVEC3(m_act_world_root_pos);
      //slogger->dump(&dlog);
      dlog.ref_force[0]       = CONV_VEC3(m_ref_force_vec[EE_INDEX_LEFT]);
      dlog.ref_force[1]       = CONV_VEC3(m_ref_force_vec[EE_INDEX_RIGHT]);
      dlog.abs_rate           = dlog::V3( m_rate.data.avx, m_rate.data.avy, m_rate.data.avz );
      dlog.accIn              = dlog::V3( m_acc.data.ax, m_acc.data.ay, m_acc.data.az );
      for(int i=0;i<12;i++){
          dlog.tau_ref[i] = (float)m_tauRef.data[i];
      }
      dlog.x_offset      = dlog::V3( m_owpg.getXOffset() );
      dlog.x_offset_orig = dlog::V3( m_owpg.getXOffsetOrig() );
      dlog.Dx_offset      = dlog::V3( m_owpg.getDXOffset() );
      dlog.Dx_offset_orig = dlog::V3( m_owpg.getDXOffsetOrig() );
      dlogger.push(dlog);
  }
#endif
  /*==================================================*/
  /* Set Target Angle Vector and publish from outport */
  /*==================================================*/
  const bool shw_debug_msg_outputdata = ((loop%500==0) || ((loop%20==0) && (m_current_control_state == PR_TRANSITION_TO_READY || m_current_control_state == PR_TRANSITION_TO_IDLE)));
  //setOutputData(shw_debug_msg_outputdata);
  setOutputData(false);

#if 0
  for(int i=0; i<m_robot->numJoints(); i++){
      if((m_ref_q[i] > 10.0)||(m_ref_q[i] < -10.0)||(std::isnan(m_ref_q[i]))||(!(std::isfinite(m_ref_q[i])))){
          std::cout << "[PR] joint(" << i << ") = " << m_ref_q[i] << " after setOutputData()" << std::endl;
      }
  }
#endif

  loop ++;
  return RTC::RTC_OK;
}; /* OnExecute() */

void PushRecover::printMembers(const int cycle){
    if(loop%4000==1) std::cout << CLEAR_CONSOLE << std::endl;
    if(loop%cycle==1){
        std::cout << MOVE_CURSOLN(0) <<  "[pr] " << MAKE_CHAR_COLOR_GREEN << "SHOW STATE"<< MAKE_CHAR_COLOR_DEFAULT << std::endl;
        std::cout << MOVE_CURSOLN(1) << "[" << m_profile.instance_name << "] rootLink_p=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_robot->rootLink()->p(0), m_robot->rootLink()->p(1), m_robot->rootLink()->p(2));
        std::cout << MOVE_CURSOLN(2) << "[" << m_profile.instance_name << "] act_zmp=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_act_zmp(0), m_act_zmp(1), m_act_zmp(2));
        std::cout << MOVE_CURSOLN(3) << "[" << m_profile.instance_name << "] m_rel_act_zmp=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_rel_act_zmp(0), m_rel_act_zmp(1), m_rel_act_zmp(2));
        std::cout << MOVE_CURSOLN(4) << "[" << m_profile.instance_name << "] rel_ref_zmp=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_rel_ref_zmp(0), m_rel_ref_zmp(1), m_rel_ref_zmp(2));
        std::cout << MOVE_CURSOLN(5) << "[" << m_profile.instance_name << "] ref_fource[0]=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_ref_force_vec[0](0), m_ref_force_vec[0](1), m_ref_force_vec[0](2));
        std::cout << MOVE_CURSOLN(6) << "[" << m_profile.instance_name << "] ref_fource[1]=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_ref_force_vec[1](0), m_ref_force_vec[1](1), m_ref_force_vec[1](2));
        std::cout << MOVE_CURSOLN(7) << "[" << m_profile.instance_name << "] act_rtp=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_act_root_pos(0), m_act_root_pos(1), m_act_root_pos(2));
        std::cout << "[" << m_profile.instance_name << "] act_cog=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_act_cog(0), m_act_cog(1), m_act_cog(2));
        std::cout << "[" << m_profile.instance_name << "] ref_cog=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_ref_cog(0), m_ref_cog(1), m_ref_cog(2));
        std::cout << "[" << m_profile.instance_name << "] act_cogvel=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_act_cogvel(0), m_act_cogvel(1), m_act_cogvel(2));
        std::cout << "[" << m_profile.instance_name << "] est_cogvel_from_rpy=[";
        printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_est_cogvel_from_rpy(0), m_est_cogvel_from_rpy(1), m_est_cogvel_from_rpy(2));

        printf("[pr] contact_st=[");
        if(m_contactStates.data[ee_index_lr[EE_INDEX_LEFT]]){
            printf("true , ");
        }else{
            printf("false, ");
        }
        if(m_contactStates.data[ee_index_lr[EE_INDEX_RIGHT]]){
            printf("true ]\n");
        }else{
            printf("false]\n");
        }
        printf("[pr] waling_st=[");
        if(m_walkingStates.data){
            printf("true ]\n");
        }else{
            printf("false]\n");
        }
        printf("[pr] swing_tim=[");
        printf("%+1.1lf, %+1.1lf]\n",
               m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_RIGHT]],
               m_controlSwingSupportTime.data[ee_index_lr[EE_INDEX_LEFT]]);
        printf("[pr] sbpCogoft=[");
        printf("%+3.4lf, %+3.4lf, %+3.4lf]\n",m_sbpCogOffset.data.x,m_sbpCogOffset.data.y,m_sbpCogOffset.data.z);
        printf("\n");
        printf("[pr] target_q=[");
        for(int i=0;i< 12;i++){
            printf("%+3.2lf, ",rad2deg(m_robot->joint(i)->q));
        }
        printf("]\n");
        printf("[pr] prev_rfq=[");
        for(int i=0;i< 12;i++){
            printf("%+3.2lf, ",rad2deg(m_prev_ref_q[i]));
        }
        printf("]\n");
        printf("[pr] curren_q=[");
        for(int i=0;i< 12;i++){
            printf("%+3.2lf, ",rad2deg(m_qCurrent.data[i]));
        }
        printf("]\n");
        printf("[pr] prev_target_dq=[");
        for(int i=0;i< 12;i++){
            printf("%+3.2lf, ",rad2deg(m_robot->joint(i)->dq));
        }
        printf("]\n");
    }
};

/*
RTC::ReturnCode_t PushRecover::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


bool PushRecover::assertEmergencyStop(void){
    if(emergencyStopReqFlag){
        return false;
    }else{
        emergencyStopReqFlag = true;
    }
    return true;
}

bool PushRecover::shiftPushRecoveryState(PushRecoveryState target_state){
    double start_ratio, goal_ratio;
    const double transition_time = 2.0;

    /* assure that interpolator is not currently working  */
    if(!transition_interpolator->isEmpty()){
        std::cerr << "[" << m_profile.instance_name << "] in " << __func__ << ", \"interpolator is currently working\"" << std::endl;
        return false;
    }

    Guard guard(m_mutex);

    /* set current interpolator ratio as start ratio to prevent interpolation jump */
    transition_interpolator->get(&start_ratio, true);

    switch(target_state){
    case PR_IDLE:
        /* Copy current joint angle in store buffer to keep the interpolater start condition */
        for(std::size_t i=0; i<m_robot->numJoints(); i++){
            m_ref_q_store[i] = m_prev_ref_q[i];
        }
    case PR_TRANSITION_TO_IDLE:
        /* transit to idle state */
        //start_ratio = 1.0;
        goal_ratio  = 0.0;
        m_current_control_state = PR_TRANSITION_TO_IDLE;
        break;
    case PR_READY:
    case PR_TRANSITION_TO_READY:
        /* transit to ready state */
        //start_ratio = 0.0;
        goal_ratio  = 1.0;
        m_current_control_state = PR_TRANSITION_TO_READY;
        break;
    default:
        start_ratio = goal_ratio = 0.0;
        return false;
        break;
    }

    /* Setup interpolator */
    transition_interpolator->clear();
    transition_interpolator->set(&start_ratio);
    {
        /* Enable Interpolator */
        const bool interpolate_immediate_flag = true;
        transition_interpolator->go(&goal_ratio, transition_time, interpolate_immediate_flag);
    }
    return true;
}

void PushRecover::waitPushRecoveryStateReady(void){
    /* TODO */
    {
        //Guard guard(m_mutex_busy_state);
    }
}

bool PushRecover::startPushRecovery(void){
    bool result;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << std::endl;

    switch(m_current_control_state){
    case PR_IDLE:
        result = shiftPushRecoveryState(PR_READY);
        break;
    default:
        result = false;
        break;
    }

    /* Show Message when failed */
    if(!result){
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ", failed to start push recovery. Please try again later." << std::endl;
    }

    return result;
}

bool PushRecover::stopPushRecovery(void){
    bool result;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << std::endl;

    switch(m_current_control_state){
    case PR_IDLE:
    case PR_TRANSITION_TO_IDLE:
        /* nothing to do */
        result = true;
        break;
    case PR_READY:
        result = shiftPushRecoveryState(PR_IDLE);
        break;
    case PR_TRANSITION_TO_READY:
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << " Currently moving to start push recovery. Please try again later." << std::endl;
        result = false;
        break;
    case PR_BUSY:
        waitPushRecoveryStateReady(); /* sleep during state is busy */
        result = shiftPushRecoveryState(PR_IDLE);
        break;
    default:
        result = false;
        break;
    }

    disablePushDetect();

    /* Show Message when failed */
    if(!result){
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ", failed to stop push recovery." << std::endl;
    }

    return result;
}

bool PushRecover::startLogging(void){
    bool result = true;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << std::endl;

    //Guard guard(m_mutex);
#ifdef USE_DATALOG
#if 1 // dlogger version
    result =  dlogger.startDumpFile();
    return result;
#else // slogger version
    if(!slogger->isRunning()){
        if(slogger->startLogging(true)){
            dlog_save_flag = true;
            std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": start logger" << std::endl;
            result = true;
        }else{
            std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": could not start logger" << std::endl;
            dlog_save_flag = false;
            result         = false;
        }
    }else{
        dlog_save_flag = false;
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": already running logger" << std::endl;
        result = false;
    }
    return result;
#endif
#else
    return true;
#endif /* USE_DATALOG */
}

bool PushRecover::stopLogging(void){
    bool result;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << "\n This function is obsoleted." << std::endl;
#if 0
    Guard guard(m_mutex);
    if(slogger->isRunning()){
        slogger->stopLogging();
        dlog_save_flag = false;
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": stopped logger" << std::endl;
        result = true;
    }else{
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": There is no running logger" << std::endl;
        result = false;
    }
#endif
    return result;
}

bool PushRecover::enablePushDetect(void){
    std::cerr << "[" << m_profile.instance_name << "] " << MAKE_CHAR_COLOR_RED << "Enabling Push Detection" << MAKE_CHAR_COLOR_DEFAULT << std::endl;

    if(m_current_control_state == PR_READY){
        pushDetector_state = PD_ENABLE;
    }else{
        pushDetector_state = PD_DISABLE;
        return false;
    }

    return true;
}

bool PushRecover::disablePushDetect(void){
    std::cerr << "[" << m_profile.instance_name << "] " << MAKE_CHAR_COLOR_RED << "Disabling Push Detection" << MAKE_CHAR_COLOR_DEFAULT << std::endl;

    pushDetector_state = PD_DISABLE;

    return true;
}

bool PushRecover::setPushDetectParam(const OpenHRP::PushRecoverService::PushDetectParam& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] setPushDetectParam" << std::endl;
    pushDetectParam = i_param;

    std::cout << "[pr] diff_margin_threshold_x=[" << pushDetectParam.diff_margin_threshold_x << "]" << std::endl;
    std::cout << "[pr] diff_margin_threshold_y=[" << pushDetectParam.diff_margin_threshold_y << "]" << std::endl;
    std::cout << "[pr] body_compliance_k=[" << pushDetectParam.body_compliance_k1 << ", " << pushDetectParam.body_compliance_k2 << ", " << pushDetectParam.body_compliance_k3 << "]" << std::endl;

    return true;
}

bool PushRecover::getPushDetectParam(OpenHRP::PushRecoverService::PushDetectParam& o_param)
{
    std::cerr << "[" << m_profile.instance_name << "] getPushDetectParam" << std::endl;
    o_param.diff_margin_threshold_x = pushDetectParam.diff_margin_threshold_x;
    o_param.diff_margin_threshold_y = pushDetectParam.diff_margin_threshold_y;
    o_param.body_compliance_k1      = pushDetectParam.body_compliance_k1;
    o_param.body_compliance_k2      = pushDetectParam.body_compliance_k2;
    o_param.body_compliance_k3      = pushDetectParam.body_compliance_k3;
    o_param.x_gain_0                = pushDetectParam.x_gain_0;
    o_param.x_gain_1                = pushDetectParam.x_gain_1;
    o_param.dx_gain_0               = pushDetectParam.dx_gain_0;
    o_param.dx_gain_1               = pushDetectParam.dx_gain_1;

    return true;
}

bool PushRecover::setOnlineWalkParam(const OpenHRP::PushRecoverService::OnlineWalkParam& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] setOnlineWalkParam" << std::endl;

    m_modify_rot_context.onlineWalkParam = i_param;
    //m_owpg.setMaxCalclen( i_param.owpg_step_time + 100 + i_param.datal );
    //m_modify_rot_context.foot_roll_gain = i_param.dataf;
    //m_owpg.modifyFirstMagnity( (0.001*i_param.datal) );

    std::cerr << "[" << m_profile.instance_name << "] filter_fp=" << m_modify_rot_context.onlineWalkParam.filter_fp << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] lpf_fp=" << m_modify_rot_context.onlineWalkParam.lpf_fp << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] modify_rot_gain_x=" << m_modify_rot_context.onlineWalkParam.modify_rot_gain_x << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] modify_rot_gain_y=" << m_modify_rot_context.onlineWalkParam.modify_rot_gain_y << std::endl;

    std::cerr << "[" << m_profile.instance_name << "] step_time=" << m_modify_rot_context.onlineWalkParam.owpg_step_time << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] step_margin=" << m_modify_rot_context.onlineWalkParam.owpg_step_margin << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] iterate_num=" << m_modify_rot_context.onlineWalkParam.owpg_iterate_num << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] feedback_gain=" << m_modify_rot_context.onlineWalkParam.owpg_feedback_gain << std::endl;

#if !defined(USE_ROBUST_ONLINE_PATTERN_GENERATOR)
    m_modify_rot_context.copyWalkParam( &m_owpg );
#else
    m_modify_rot_context.copyWalkParam( &m_owpg );
    m_abs_est.setGyroFilterParam( m_modify_rot_context.onlineWalkParam.filter_fp );
    m_owpg.setFirstMagnify( (float)i_param.owpg_modify_first_magnity );
#endif
    return true;
};

bool PushRecover::getOnlineWalkParam(OpenHRP::PushRecoverService::OnlineWalkParam& o_param)
{
    std::cerr << "[" << m_profile.instance_name << "] getOnlineWalkParam" << std::endl;
    m_modify_rot_context.copyWalkParam( &o_param );
    return true;
};

bool PushRecover::setWheelMode(const long mode){
    std::cerr << "[" << m_profile.instance_name << "] setWheelMode(" << mode << ")" << std::endl;
    return m_wheel_ctrl->setControlMode(mode);
};

bool PushRecover::setWheelControllerParam(const OpenHRP::PushRecoverService::WheelControllerParamSet& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] setWheelControllerParam" << std::endl;
    return m_wheel_ctrl->setParam(i_param);
};

bool PushRecover::getWheelControllerParam(OpenHRP::PushRecoverService::WheelControllerParamSet& o_param)
{
    std::cerr << "[" << m_profile.instance_name << "] getWheelControllerParam" << std::endl;
    return m_wheel_ctrl->getParam(o_param);
    //o_param.drivenum = 2;
    //o_param.param.length(2);
    // o_param.param[0].enable = true;
    // o_param.param[1].enable = true;
    // o_param.param[0].pgain = 1.0;
    // o_param.param[1].pgain = 1.0;
    // o_param.param[0].dgain = 1.0;
    // o_param.param[1].dgain = 1.0;
    //return true;
};



extern "C"
{

  void PushRecoverInit(RTC::Manager* manager)
  {
    RTC::Properties profile(pushrecover_spec);
    manager->registerFactory(profile,
                             RTC::Create<PushRecover>,
                             RTC::Delete<PushRecover>);
  }

};
