#include "uvc_walker/uvc_walker.hpp"
using std::placeholders::_1;

namespace {
constexpr float TWO_PI = 6.283185307179586f;
constexpr float RAD_PER_TICK = (2.0f * M_PI) / 4096.0f;
constexpr uint16_t CENTER_TICK = 2048;
constexpr uint16_t ADDR_GOAL_POSITION = 116; // MX 2.0: Goal Position(4B)
constexpr float MAX_DELTA_SERVO_UNIT = 100.0f; // 원본 한 루프 ±100
}

UVCWalker::UVCWalker() : Node("uvc_walker")
{
  RCLCPP_INFO(get_logger(), "Init UVC Walker (Legs + Waist + Arms)");

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50, std::bind(&UVCWalker::ImuCallback, this, _1));

  timer_ = create_wall_timer(std::chrono::milliseconds(10),
                             std::bind(&UVCWalker::TimerCallback, this));

  portHandler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
  if (!portHandler_->openPort() || !portHandler_->setBaudRate(4000000)) {
    RCLCPP_ERROR(get_logger(), "DXL open @4Mbps failed");
  } else {
    RCLCPP_INFO(get_logger(), "DXL opened @4Mbps");
  }

  // 초기값
  autoH_  = HEIGHT_;
  fwct_   = 1; fwctEnd_ = 18;
  landF_  = landB_ = 0;
  swMax_  = 25; fhMax_ = 35;

  // 팔 초기 포즈: 어깨 yaw/roll = 0, 팔꿈치 약간 굽힘 권장(예: 20도)
  U0W_R_ = 0; U2W_R_ = 0; EW_R_ = 20.0f * M_PI/180.0f;
  U0W_L_ = 0; U2W_L_ = 0; EW_L_ = 20.0f * M_PI/180.0f;
  U1W_R_ = 0; U1W_L_ = 0; // pitch는 ArmCont에서 스윙
}





void UVCWalker::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const float x = msg->orientation.x;
  const float y = msg->orientation.y;
  const float z = msg->orientation.z;
  const float w = msg->orientation.w;

  roll_  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x*x + y*y));
  pitch_ = std::asin(std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f));

  // 각속도(옵션, 발목 보정용)
  roll_rate_  = msg->angular_velocity.x;
  pitch_rate_ = msg->angular_velocity.y;
}



void UVCWalker::TimerCallback()
{
  Walk();
  WriteAllTargets();
}




// ========== DXL Helper ==========
bool UVCWalker::SetPosition(uint8_t id, float rad)
{
  if (rad >  M_PI) 
    rad -= TWO_PI;
  while (rad < -M_PI) 
    rad += TWO_PI;

  const int32_t tick = static_cast<int32_t>(std::lround(CENTER_TICK + rad / RAD_PER_TICK));
  const uint32_t val = static_cast<uint32_t>(std::clamp(tick, 0, 4095));
  uint8_t dxl_error = 0;
  const int rc = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_POSITION, val, &dxl_error);
  
  if (rc != COMM_SUCCESS) 
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "DXL write fail id=%d rc=%d", id, rc);
    return false;
  }
  return true;
}




void UVCWalker::WriteAllTargets()
{
  // ===== 다리 =====
  // 오른쪽(1): 부호는 조립에 따라 조정 요망
  SetPosition(id_.thigh_yaw_R,    K2W_[1]);
  SetPosition(id_.hip_roll_R,    K1W_[1]);
  SetPosition(id_.hip_pitch_R,    HW_[1]);
  SetPosition(id_.knee_R,         K0W_[1]);
  // 발목 pitch에 IMU 각속도 피드포워드(원본 계수 사용)
  SetPosition(id_.ankle_pitch_R,  A0W_[1] + pitch_gyrg_ * pitch_rate_);
  SetPosition(id_.ankle_roll_R,   A1W_[1] - roll_gyrg_  * roll_rate_);

  // 왼쪽(0)
  SetPosition(id_.thigh_yaw_L,    K2W_[0]);
  SetPosition(id_.hip_roll_L,     K1W_[0]);
  SetPosition(id_.hip_pitch_L,    HW_[0]);
  SetPosition(id_.knee_L,         K0W_[0]);
  SetPosition(id_.ankle_pitch_L,  A0W_[0] + pitch_gyrg_ * pitch_rate_);
  SetPosition(id_.ankle_roll_L,   A1W_[0] - roll_gyrg_  * roll_rate_);

  // ===== 허리/머리 =====
  SetPosition(id_.waist_yaw, WESTW_);


  // ===== 팔 =====
  // U0 yaw / U2 roll은 기본값 유지, U1 pitch는 ArmCont 스윙 적용
  SetPosition(id_.arm_sh_yaw_R,   U0W_R_);
  SetPosition(id_.arm_sh_pitch_R, U1W_R_);
  SetPosition(id_.arm_sh_roll_R,  U2W_R_);
  SetPosition(id_.arm_elbow_R,    EW_R_);

  SetPosition(id_.arm_sh_yaw_L,   U0W_L_);
  SetPosition(id_.arm_sh_pitch_L, U1W_L_);
  SetPosition(id_.arm_sh_roll_L,  U2W_L_);
  SetPosition(id_.arm_elbow_L,    EW_L_);

  SetPosition(id_.head_yaw,  HEADW_); // 
  SetPosition(id_.head_pitch,HEADW_); // 
}

// ========== UVC Core ==========
void UVCWalker::Uvc()
{
  float rb = roll_, pb = pitch_;
  float k = std::sqrt(pitch_*pitch_ + roll_*roll_);
  if (k > 0.033f) 
    { 
        k = (k - 0.033f) / k; 
        pitch_ *= k; 
        roll_ *= k; 
    }
  else            
    { 
        pitch_ = 0.0f; 
        roll_ = 0.0f; 
    }

  rollt_  = 0.25f * roll_;
  if (jikuasi_ == 0) rollt_ = -rollt_;
  pitcht_ = 0.25f * pitch_;

  if (fwct_ > landF_ && fwct_ <= fwctEnd_ - landB_) 
  {
    // y축
    float th = std::atan((dyi_ - sw_) / autoH_);
    float kl = autoH_ / std::cos(th);
    float ks = th + rollt_;
    float s  = kl * std::sin(ks);
    dyi_  = s + sw_;
    autoH_ = kl * std::cos(ks);
    // x축
    th = std::atan(dxi_ / autoH_);
    kl = autoH_ / std::cos(th);
    ks = th + pitcht_;
    s  = kl * std::sin(ks);
    dxi_  = s;
    autoH_ = kl * std::cos(ks);

    dyi_ = std::clamp(dyi_, 0.0f, 45.0f);
    dxi_ = std::clamp(dxi_, -45.0f, 45.0f);
    dyis_ = dyi_;
    dxis_ = -dxi_;

    float ktmp, ksum;
    if (jikuasi_ == 0) { ktmp = -sw_ + dyi_;  ksum =  sw_ + dyis_; }
    else               { ksum = -sw_ + dyi_;  ktmp =  sw_ + dyis_; }
    if (ktmp + ksum < 0.0f) dyis_ -= (ktmp + ksum);
  }
  roll_  = rb;
  pitch_ = pb;
}

void UVCWalker::UvcSub()
{
  if (fwct_ <= landF_) {
    float k = dyi_ / (11.0f - fwct_);
    dyi_  -= k;
    dyis_ += k;

    const float rst = 3.0f;
    if (dxi_ > rst)       { dxi_ -= rst; dxis_ -= rst; }
    else if (dxi_ < -rst) { dxi_ += rst; dxis_ += rst; }
    else { dxis_ -= dxi_; dxi_ = 0.0f; }
  }
  dyis_ = std::clamp(dyis_, -70.0f, 70.0f);
  dxis_ = std::clamp(dxis_, -70.0f, 70.0f);

  if (HEIGHT_ > autoH_) autoH_ += (HEIGHT_ - autoH_) * 0.07f;
  else                  autoH_  = HEIGHT_;

  if (fwct_ > fwctEnd_ - landB_ && rollt_ > 0.0f) {
    autoH_ -= (std::fabs(dyi_ - dyib_) + std::fabs(dxi_ - dxib_)) * 0.02f;
  }
  if (autoH_ < 140.0f) autoH_ = 140.0f;
}

void UVCWalker::UvcSub2()
{
  float k1 = dyi_ / (fwctEnd_ - fwct_ + 1.0f);
  dyi_  -= k1;
  float k0 = dxi_ / (fwctEnd_ - fwct_ + 1.0f);
  dxi_  -= k0;

  wk_    -= wk_ / (fwctEnd_ - fwct_ + 1.0f);
  WESTW_ -= WESTW_ / (fwctEnd_ - fwct_ + 1.0f);

    K2W_[0] =  WESTW_;   // Left thigh yaw
  K2W_[1] = -WESTW_;   // Right thigh yaw

  if (fwct_ <= landF_) { dyis_ += k1; dxis_ -= k0; }
  else                 { dyis_ -= dyis_ / (fwctEnd_ - fwct_ + 1.0f);
                         dxis_ -= dxis_ / (fwctEnd_ - fwct_ + 1.0f); }

  autoH_ += (HEIGHT_ - autoH_) / (fwctEnd_ - fwct_ + 1.0f);
  dyis_ = std::clamp(dyis_, -70.0f, 70.0f);
  dxis_ = std::clamp(dxis_, -70.0f, 70.0f);
}

void UVCWalker::FootUp()
{
  if (fwct_ > landF_ && fwct_ <= (fwctEnd_ - landB_))
    fh_ = fhMax_ * std::sin(M_PI * (fwct_ - landF_) / (fwctEnd_ - (landF_ + landB_)));
  else
    fh_ = 0.0f;
}

// ---- 역기구학(원본 footCont) ----
void UVCWalker::FootCont(float x, float y, float h, int s)
{
  float klen = std::sqrt(x*x + std::pow(std::sqrt(y*y + h*h) - LegKinematics::L_offset, 2));
  if (klen > LegKinematics::K_limit) {
    autoH_ = std::sqrt(std::pow(std::sqrt(LegKinematics::K_limit*LegKinematics::K_limit - x*x) + LegKinematics::L_offset, 2) - y*y);
    klen = LegKinematics::K_limit;
  }
  const float xang = std::asin(std::clamp(x / klen, -1.0f, 1.0f));
  float kang = std::acos(std::clamp(klen / LegKinematics::L_max, -1.0f, 1.0f));
  const float kang_limit = 1800.0f / CHG_SVA_; // ≈1.046rad
  if (kang > kang_limit) kang = kang_limit;

  const float max_delta_rad = MAX_DELTA_SERVO_UNIT / CHG_SVA_;
  float hip_pitch_des = 2.0f * kang;
  hip_pitch_des = clampRadDelta(hip_pitch_des, HW_[s], max_delta_rad);

  HW_[s]  = hip_pitch_des;
  K0W_[s] = kang + xang;
  A0W_[s] = kang - xang;

  float kroll = std::atan2(y, h);
  K1W_[s] = clampRadDelta(kroll, K1W_[s], max_delta_rad);
  A1W_[s] = -K1W_[s];
}

inline float UVCWalker::clampRadDelta(float desired, float current, float max_delta_rad)
{
  float delta = desired - current;
  if (delta >  max_delta_rad) delta =  max_delta_rad;
  if (delta < -max_delta_rad) delta = -max_delta_rad;
  return current + delta;
}

// ---- 두 발 동시에 IK 적용 + 허리 회전 산출 ----
void UVCWalker::FeetCont1(float x0, float y0, float x1, float y1, int s)
{
  if (s == 1) {
    if (std::fabs(y0 + 21.5f) < 1e-6f) wt_ = 0.0f;
    else if (jikuasi_ == 0) { wt_ = 0.5f * std::atan(x0 / (y0 + 21.5f)); wk_ = std::fabs(15.0f * x0 / 45.0f); }
    else                    { wt_ = 0.5f * std::atan(-x1 / (y1 + 21.5f)); wk_ = std::fabs(15.0f * x1 / 45.0f); }
    WESTW_ = wt_; // 허리 요(라디안)
  }
  if (jikuasi_ == 0) {
    FootCont(x0, y0 - wk_, autoH_    , 0);
    FootCont(x1, y1 - wk_, autoH_ - fh_, 1);
  } else {
    FootCont(x0, y0 - wk_, autoH_ - fh_, 0);
    FootCont(x1, y1 - wk_, autoH_    , 1);
  }
}

// ---- 좌표 배치 ----
void UVCWalker::FeetCont2(int s)
{
  if (jikuasi_ == 0) {
    FeetCont1(dxi_  - swx_, dyi_  - swy_,
              dxis_ - swx_, dyis_ + swy_, s);
  } else {
    FeetCont1(dxis_ - swx_, dyis_ + swy_,
              dxi_  - swx_, dyi_  - swy_, s);
  }
}

void UVCWalker::CounterCont()
{
  if (fwct_ >= fwctEnd_) {
    jikuasi_ ^= 1;
    fwct_ = 0;
    fh_ = 0;
    std::swap(dyis_, dyi_);  dyib_ = dyi_;
    std::swap(dxis_, dxi_);  dxib_ = dxi_;
  } else {
    fwct_ += 1.0f;
    if (fwct_ > fwctEnd_) fwct_ = fwctEnd_;
  }
}

// ---- ★ 팔 스윙(원본 armCont) ----
void UVCWalker::ArmCont()
{
  // 원본: U1W[0] = 510*dyis/70; if(U1W[0]<0)U1W[0]=0; U1W[1]=U1W[0];
  // → 라디안 환산: 서보 510단위 ≈ 510/CHG_SVA_ rad
  float arm_pitch = (510.0f * dyis_ / 70.0f) / CHG_SVA_;
  if (arm_pitch < 0.0f) arm_pitch = 0.0f;

  // 좌/우 동일 크기로 스윙—조립에 맞춰 부호 조정 가능
  U1W_L_ =  arm_pitch;
  U1W_R_ = -arm_pitch; // 좌우 상쇄 스윙(반대 위상) 권장
}

// ========== 메인 상태머신 ==========
void UVCWalker::Walk()
{
  switch (mode_) {
    case 710: {
      if (!ready_logged_) { RCLCPP_INFO(get_logger(), "Mode 710 (Ready)"); ready_logged_ = true; }
      if (--motCt_ <= 0) {
        dxi_=dyi_=dxis_=dyis_=dxib_=dyib_=0;
        landF_=landB_=0;
        fwct_=1; fwctEnd_=18;
        autoH_=HEIGHT_;
        sw_=swx_=swy_=0;
        jikuasi_=1;
        FootCont(0,0,HEIGHT_,0);
        jikuasi_=0;
        FootCont(0,0,HEIGHT_,1);
        mode_ = 740;
        RCLCPP_INFO(get_logger(), "Mode 740 (Start Walking)");
      }
    } break;

    case 740: {
      Uvc();
      UvcSub();
      FootUp();
      FeetCont2(1);
      ArmCont();           // ★ 팔 스윙
      CounterCont();
      if (fwct_ == 0.0f) { mode_ = 750; RCLCPP_INFO(get_logger(), "Mode 750 (Settle)"); }
    } break;

    case 750: {
      FeetCont2(1);
      ArmCont();
      if (fwct_ > 30.0f) {
        fwct_ = 1.0f;
        const float k = std::sqrt(0.5f*dxis_*dxis_ + dyis_*dyis_);
        swMax_ = 17.0f + 17.0f * k / 45.0f;
        mode_ = 760;
        RCLCPP_INFO(get_logger(), "Mode 760 (Main Steps) swMax=%.1f", swMax_);
      } else { fwct_ += 1.0f; }
    } break;

    case 760: {
      landF_ = 25.0f;
      fwctEnd_ = landF_ + 25.0f;
      UvcSub2();
      FootUp();

      const float k = swMax_ * std::sin(M_PI * fwct_ / fwctEnd_);
      const float t = std::atan((std::fabs(dxi_) + std::fabs(21.5f * std::sin(wt_)))
                              / (dyi_ + 21.5f * std::cos(wt_) - wt_));
      swx_ = (dxi_ > 0) ?  (k * std::sin(t)) : (-k * std::sin(t));
      swy_ = k * std::cos(t);

      FeetCont2(0);
      ArmCont();
      CounterCont();

      if (fwct_ == 0.0f) {
        dxi_=dyi_=dxis_=dyis_=dxib_=dyib_=0;
        landF_=landB_=0;
        fwct_=1; fwctEnd_=18;
        autoH_=HEIGHT_;
        sw_=swx_=swy_=0;
        jikuasi_=1;
        mode_ = 770;
        RCLCPP_INFO(get_logger(), "Mode 770 (End settle)");
      }
    } break;

    case 770: {
      FeetCont2(0);
      ArmCont();
      fwct_ += 1.0f;
      if (fwct_ > 50.0f) {
        fwct_ = 1.0f; mode_ = 710; ready_logged_ = false;
        RCLCPP_INFO(get_logger(), "Back to 710");
      }
    } break;
  }
}
