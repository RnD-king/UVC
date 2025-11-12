#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <algorithm>
#include <array>
#include "dynamixel_sdk/dynamixel_sdk.h"

// ====== 기구 파라미터 ======
struct LegKinematics {
  static constexpr float L_offset = 63.3f;
  static constexpr float L_max    = 250.0f;
  static constexpr float K_limit  = 249.0f;
};




// ====== Dynamixel ID 매핑 (22축 완전판) ======
struct DxlMap {
  // 오른쪽 다리
  uint8_t thigh_yaw_R    = 10;  // K2R
  uint8_t hip_roll_R     =  8;  // K1
  uint8_t hip_pitch_R    =  6;  // HW
  uint8_t knee_R         =  4;  // K0
  uint8_t ankle_pitch_R  =  2;  // A0
  uint8_t ankle_roll_R   =  0;  // A1
  // 왼쪽 다리
  uint8_t thigh_yaw_L    = 11;  // K2L
  uint8_t hip_roll_L     =  9;
  uint8_t hip_pitch_L    =  7;
  uint8_t knee_L         =  5;
  uint8_t ankle_pitch_L  =  3;
  uint8_t ankle_roll_L   =  1;

  // 허리/머리
  uint8_t waist_yaw      = 12;  // WEST

  // 오른팔 (U0/U1/U2/EW)
  uint8_t arm_sh_yaw_R   = 14;
  uint8_t arm_sh_pitch_R = 16;
  uint8_t arm_sh_roll_R  = 18;
  uint8_t arm_elbow_R    = 20;
  // 왼팔
  uint8_t arm_sh_yaw_L   = 13;
  uint8_t arm_sh_pitch_L = 15;
  uint8_t arm_sh_roll_L  = 17;
  uint8_t arm_elbow_L    = 19;
  //HEAD
  uint8_t head_yaw       = 21;  // HEAD1
  uint8_t head_pitch     = 22;  // HEAD2
};

class UVCWalker : public rclcpp::Node
{
public:
  UVCWalker();

private:
  // ===== ROS2 =====
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void TimerCallback();
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ===== Dynamixel =====
  dynamixel::PortHandler*  portHandler_{nullptr};
  dynamixel::PacketHandler* packetHandler_{nullptr};
  bool SetPosition(uint8_t id, float rad);
  void WriteAllTargets();
  std::vector<uint8_t> dxl_ids_; 

  // ===== IMU =====
  float roll_{0.0f}, pitch_{0.0f};
  float roll_rate_{0.0f}, pitch_rate_{0.0f};

  // ===== 보행 상태 =====
  int   mode_{710};
  int   jikuasi_{0};
  int   motCt_{100};
  float HEIGHT_{300.0f};


  float CHG_SVA_{1718.9f};
  float pitch_gyrg_{0.08f}, roll_gyrg_{0.10f};
  float dxi_{0}, dyi_{0}, dxis_{0}, dyis_{0};
  float dxib_{0}, dyib_{0};
  float autoH_{HEIGHT_};
  float rollt_{0}, pitcht_{0};
  float fwct_{1}, fwctEnd_{18};
  float fh_{0}, fhMax_{50};
  float sw_{0}, swx_{0}, swy_{0}, swMax_{25};
  float landF_{0}, landB_{0};
  float wk_{0}, wt_{0};
  float WESTW_{0}, HEADW_{0};



      // 균형용 내부 상태
    float p_ofs_ = 0.0f;
    float r_ofs_ = 0.0f;
    float ipa_ = 0.0f, ira_ = 0.0f;
    float ip_ = 0.0f, ir_ = 0.0f;


  // ===== 관절 변수 =====
  std::array<float,2> HW_{0,0};
  std::array<float,2> K0W_{0,0};
  std::array<float,2> A0W_{0,0};
  std::array<float,2> K1W_{0,0};
  std::array<float,2> A1W_{0,0};
  std::array<float,2> K2W_{0,0};  // ★ 추가

  float U0W_R_{0}, U1W_R_{0}, U2W_R_{0}, EW_R_{0};
  float U0W_L_{0}, U1W_L_{0}, U2W_L_{0}, EW_L_{0};

  // ===== 핵심 로직 =====
  void Walk();        // 전체 보행 루프 (메인 제어)
  void Uvc();         // 상체 수직 제어 (IMU 기반 균형 유지)
  void UvcSub();      // UVC 보조 제어 1 (지지발 중심 자세 보정)
  void UvcSub2();     // UVC 보조 제어 2 (자이로 감쇠 보정)
  void FootUp();      // 스윙발 들어올리기
  void FootCont(float x, float y, float h, int s);   // 한쪽 발 위치 제어 (x,y,h)
  void FeetCont1(float x0, float y0, float x1, float y1, int s); // 양발 균형 제어 1단계
  void FeetCont2(int s);      // 양발 균형 제어 2단계 (착지 안정화)
  void CounterCont();         // 보행 프레임/카운터 관리
  void ArmCont();             // 팔 흔들기 제어 (밸런스용)
  inline float clampRadDelta(float desired, float current, float max_delta_rad); // 관절 회전속도 제한
  bool ready_logged_{false};  // 초기화 로그 1회만 출력


  bool init_730_done_ = false;
  bool imu_ready_ = false;
  bool ik_ready_  = false;

  DxlMap id_;
};
