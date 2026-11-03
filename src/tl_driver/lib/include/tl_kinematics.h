/**
 * @file tl_kinematics.h
 * @brief 本地运动学解算接口（机器人基坐标系正逆解，支持工具手变换与按控制器参数建模）。
 * @note 1) 关节角与关节限位一律为度 (deg)；位姿为 6 元素 [X, Y, Z, A, B, C]，位移 mm、姿态角
 *       rad，旋转序 R = Rx(A) * Ry(B) * Rz(C)。
 *       2) 只处理机器人基坐标系（BASE）位姿；工具手（TCP）变换见 Kinematics::set_tool，工具系 /
 *       用户系换算请用 convert_cartesian_coord / get_origin_coord_to_target_coord。
 *       3) 轴数在编译期固定：仅 6 轴与 7 轴两套实例（Kinematics<6> / Kinematics<7>）。
 *       4) 外部轴（协议点位第 7 位）不参与本地解算。
 *       5) 分支选解、臂角（ψ）与限位规则详见《SDK 使用说明文档》「本地运动学接口」章节。
 * @attention 除 make_model 外均为纯本地计算：不连接控制器、不发送报文；make_model 读取控制器参数
 * （不下发指令），需要已连接的控制器。
 */
#ifndef TL_SDK_TL_KINEMATICS_H
#define TL_SDK_TL_KINEMATICS_H

#include <array>
#include <limits>
#include <vector>
#include "tl_types.h"

namespace tl
{

/**
 * @brief 关节限位 (deg)。
 * @note 数组长度 = 求解器轴数。
 */
struct JointLimit
{
  /** 下限 (deg) */
  double min{0};
  /** 上限 (deg)，须大于 min */
  double max{0};
};

/**
 * @brief 标准 DH 连杆参数（非改进型）。
 */
struct DHLink
{
  /** 连杆长度 a (mm) */
  double a{0};
  /** 关节扭转角 alpha (deg) */
  double alpha{0};
  /** 关节偏距 d (mm) */
  double d{0};
  /** 关节零位偏置 theta (deg) */
  double theta{0};
};

/**
 * @brief 本地运动学的求解状态（正逆解统一的返回值）。
 * @note 1) 正逆解统一以本枚举为返回值（成功 = OK / SINGULAR，失败 = 其余取值），不用 Result 码；
 *       2) SINGULAR 属成功路径：腕奇异邻域仍有合法解，此处仅标记以便调用方规避关节速率放大；
 *       3) NOT_CONVERGED 指数值精修未收敛，「位置不可达」与「不收敛」难以完全区分；
 *       4) PARAM_ERR 表示入参 / 模型非法，此时正逆解都不改写出参。
 */
enum class KinematicsStatus
{
  OK = 0,   // 成功：解在限位内且非奇异邻域
  SINGULAR, // 成功：处于腕奇异邻域（6 轴 |sin(q5)| < 1e-6；7 轴 |sin(q6)| < 1e-6），解仍有效
  UNREACHABLE,    // 失败：目标位置超出可达范围（闭式分支不存在）
  NO_SOLUTION,    // 失败：指定 form 的分支无解
  LIMIT_EXCEEDED, // 失败：存在几何解但全部候选越界
  NOT_CONVERGED,  // 失败：数值精修未收敛
  PARAM_ERR = -1, // 失败：入参 / 模型非法
};

/**
 * @brief 本地运动学求解模型：由 Kinematics<DOF> 持有。
 * @note 内部数据，用户侧无须构造；各数组长度 = 轴数。
 * @tparam DOF 轴数，取 6 或 7
 */
template <int DOF>
struct KinematicsModel
{
  /** 连杆参数 */
  std::array<DHLink, DOF> links{};
  /** 关节限位 (deg)；仅 has_limits 为 true 时有效 */
  std::array<JointLimit, DOF> limits{};
  /** cos(alpha) */
  std::array<double, DOF> cos_alpha{};
  /** sin(alpha) */
  std::array<double, DOF> sin_alpha{};
  /** 是否校验关节限位 */
  bool has_limits{false};
  /** 是否合法（见 Kinematics::is_valid） */
  bool valid{false};
  /** 工具手变换位姿 [X, Y, Z, A, B, C]（mm + rad）；全零 = 无工具手 */
  std::array<double, 6> tool{};
};

/**
 * @brief ψ 可行区间的一个端点原因（NONE = -1 为无效占位）。
 */
enum class ArmAngleBound
{
  NONE = -1,   // 无效占位（出参初始化用）
  WRAP = 0,    // 区间边界是 ψ 的 ±π 绕组边界，非约束造成
  JOINT_LIMIT, // 某关节限位绑定该端点，关节号见 lo_joint / hi_joint
  SINGULAR,    // 腕奇异绑定该端点；端点处无解或无法归因时也保守记此值
};

/**
 * @brief ψ 可行区间（闭区间，单位 rad；ψ 按 2π 周期等价）。
 * @note 下界 / 上界各自带端点原因与（关节限位时）绑定关节号。
 */
struct ArmAngleRange
{
  double lo{0};
  double hi{0};
  int form{0}; // 该区间所属的 form（1..8）
  ArmAngleBound lo_reason{ArmAngleBound::NONE};
  ArmAngleBound hi_reason{ArmAngleBound::NONE};
  int lo_joint{-1}; // 仅 lo_reason == JOINT_LIMIT 有意义，否则 -1
  int hi_joint{-1};
};

/** 全解支数上限（6 轴 8 个分支 / 7 轴给定 ψ 的 8 支）。 */
constexpr int kMaxBranches = 8;

/** 单个 form 的可行 ψ 段数参考上限（非硬性约束）。 */
constexpr int kMaxArmAngleRangesPerForm = 4;

/** ik_arm_angle_samples 的采样点数上限；超出返回 PARAM_ERR。 */
constexpr int kMaxSamplePoints = 1 << 16;

/** 「无臂角」占位（NaN）：6 轴解的 arm_angle_rad 恒为此值，也是 ik / ik_all 的默认入参
 *  （非有限值 = 未指定臂角）。 */
constexpr double kArmAngleNone = std::numeric_limits<double>::quiet_NaN();

/**
 * @brief 单条候选解（6 / 7 轴的 ik_all、7 轴的 ik_arm_angle_samples 出参）。
 * @note 1) q 为规范代表：配置限位时取限位窗口内的等价表示，未配置限位时折到 (-π, π]；
 *       2) arm_angle_rad 在 6 轴无臂角概念，恒为 kArmAngleNone。
 */
struct IKSolution
{
  std::array<double, 7> q{}; // 关节角 (deg，前 axes 项有效)
  int form{0};               // 形态编号（1..8，定义见 Kinematics::ik）
  int axes{7}; // q 的有效长度（= 求解器轴数；6 轴为 6，第 7 项恒为 0）
  double arm_angle_rad{kArmAngleNone}; // 该解对应的 ψ (rad)
  bool wrist_singular{false}; // 是否处于腕奇异邻域（解仍有效，同 KinematicsStatus::SINGULAR 语义）
};

/**
 * @brief 本地运动学求解器（轴数编译期固定）。
 * @note 1) 仅 6 轴与 7 轴两套实例，其他轴数在编译期报错；
 *       2) 构造时接管连杆参数与限位；工具手由 set_tool 设置（默认恒等 = 无工具手）；
 *       3) 除 set_tool 外对象无可变状态：const 方法可并发调用、不分配堆内存、不抛异常（出参为
 *       std::vector 的入口除外）；set_tool 改写内部状态，不得与并发的解算调用同时进行。
 * @tparam DOF 轴数，取 6 或 7
 * @attention 构造不校验入参合法性（不抛异常），非法模型由 is_valid() 报告、调用时返回 PARAM_ERR。
 */
template <int DOF>
class Kinematics
{
public:
  static_assert(DOF == 6 || DOF == 7, "本地运动学仅提供 6 轴与 7 轴实例");

  /** 轴数 */
  static constexpr int axes = DOF;
  /** 连杆参数数组（长度 = 轴数） */
  using Links = std::array<DHLink, DOF>;
  /** 关节限位数组（长度 = 轴数） */
  using Limits = std::array<JointLimit, DOF>;
  /** 关节角数组 (deg，长度 = 轴数) */
  using Joints = std::array<double, DOF>;
  /** 位姿数组 [X, Y, Z, A, B, C]（mm + rad） */
  using Poses = std::array<double, 6>;

  /**
   * @brief 默认构造空模型（is_valid() 为假，正逆解一律返回 PARAM_ERR）。
   */
  Kinematics() = default;

  /**
   * @brief 由连杆参数构造（不校验关节限位）。
   * @param[in] links 连杆参数（标准 DH，长度 = 轴数）
   */
  explicit Kinematics(const Links& links) noexcept;

  /**
   * @brief 由连杆参数与关节限位构造。
   * @param[in] links 连杆参数（标准 DH，长度 = 轴数）
   * @param[in] limits 关节限位 (deg，长度 = 轴数)
   */
  Kinematics(const Links& links, const Limits& limits) noexcept;

  /**
   * @brief 模型是否合法。
   * @note 判据：连杆参数为有限值；带限位时每轴 min < max 且限位为有限值。不合法时构造照常完成，
   * 但正逆解一律返回 PARAM_ERR。
   * @return true 合法
   */
  bool is_valid() const noexcept;

  /**
   * @brief 取连杆参数。
   * @return 连杆参数数组
   */
  const Links& links() const noexcept;

  /**
   * @brief 是否校验关节限位。
   * @return true 表示逆解会做限位校验
   */
  bool has_limits() const noexcept;

  /**
   * @brief 设置工具手变换（TCP 相对法兰）。
   * @note 1) 入参是工具末端坐标系在法兰坐标系中的位姿，6 元素 [X, Y, Z, A, B, C]（位移 mm、姿态角
   *       rad）；全零表示无工具手（恒等变换，也是构造后的默认值）；
   *       2) 设置后 fk 输出工具末端（TCP）位姿；ik / ik_all /
   *       ik_arm_angle_samples / arm_angle_ranges 的位姿入参按工具末端位姿解释；
   *       arm_angle_of 与工具手无关；
   *       3) 本接口改写对象内部状态：不得与并发的解算调用同时进行。
   * @param[in] tool 工具手位姿 [X, Y, Z, A, B, C]（mm + rad），全零 = 无工具手
   * @return OK=成功；PARAM_ERR=入参含非有限值（此时不改写当前工具手）
   */
  KinematicsStatus set_tool(const Poses& tool) noexcept;

  /**
   * @brief 取当前工具手变换。
   * @return 工具手位姿 [X, Y, Z, A, B, C]（mm + rad）；未设置时全零
   */
  const Poses& tool() const noexcept;

  /**
   * @brief 正运动学解算：关节角 → 基坐标系位姿。
   * @note 1) 纯计算，不校验关节是否在限位内（正解在限位外同样有定义）；
   *       2) 模型量级异常时返回 PARAM_ERR，不外泄 inf / NaN；
   *       3) 设置了工具手时为工具末端位姿、未设置时为法兰位姿（见 set_tool）。
   * @param[in] joints 关节角 (deg，长度 = 轴数)
   * @param[out] pose 位姿，6 元素 [X, Y, Z, A, B, C]（mm + rad）；未成功时不被改写
   * @return OK=成功；PARAM_ERR=入参 / 模型非法（正解不涉及分支与限位校验）
   */
  KinematicsStatus fk(const Joints& joints, Poses& pose) const noexcept;

  /**
   * @brief 逆运动学解算：基坐标系位姿 → 关节角（指定参考关节角）。
   * @note 1) form = 0 为推荐路径：取限位内离 ref_joints 最近的候选；
   *       2) form ∈ [1,8] 为分支编号 4*肩 + 2*肘 + 腕 + 1（三位各取 0/1，取值 1..8；逐值含义见
   *       《SDK 使用说明文档》「本地运动学接口」）；
   *       3) 6 轴标准球腕与 7 轴 S-R-S 走闭式解（不受参考点远近限制，能区分位置不可达与候选全部
   *       越限），其余结构走参考点单种子数值路径、属局部收敛（失败返回 NOT_CONVERGED 或
   *       LIMIT_EXCEEDED）；
   *       4) 结构不在闭式覆盖内时 form ≠ 0 返回 NO_SOLUTION；「存在几何解但全部候选越限」返回
   *       LIMIT_EXCEEDED，优先于 NO_SOLUTION；
   *       5) ref_joints 可传任意 2kπ 等价表示；配置限位时参考点与解都先折到限位窗口内；
   *       6) 失败时不改写 joints；OK / SINGULAR 均属成功，照常写出。
   * @param[in] pose 目标位姿，6 元素 [X, Y, Z, A, B, C]（mm + rad）；工具手语义见 set_tool
   * @param[in] ref_joints 参考关节角 (deg，长度 =
   * 轴数)，用于自动选解与等效角归一；无默认值（必填，避免
   * 选解被静默锚定在某个隐含参考点），离线一次性求解可传关节角零位
   * @param[in] form 形态：0 = 自动（取离参考点最近者）；1~8 = 指定分支（编号定义见上）
   * @param[in] arm_angle_rad 臂角 ψ (rad)，仅 7 轴有意义；默认 kArmAngleNone（NaN）表示不指定臂角。
   * 给出有限值时为硬约束：7 轴按该 ψ 求解，6 轴因无冗余自由度返回 PARAM_ERR（不静默忽略）
   * @param[out] joints 关节角 (deg，长度 = 轴数)；未成功时不被改写
   * @return OK=成功；SINGULAR=成功且处于腕奇异邻域（解有效）；其余取值为失败（区分见
   * KinematicsStatus）；PARAM_ERR=入参 / 模型非法（模型不合法、位姿或参考角含非有限值、form
   * 越界、6 轴指定臂角）
   */
  KinematicsStatus ik(const Poses& pose, Joints& joints, const Joints& ref_joints, int form = 0,
                      double arm_angle_rad = kArmAngleNone) const noexcept;

  /**
   * @brief 关节角 → 臂角 ψ（仅 7 轴）。
   * @note 1) 本接口是 ψ 定义的唯一权威实现；ψ 取值域 [-π, π)（rad，任意 2π 等价表示合法）；
   *       2) ψ 无定义时返回 NO_SOLUTION：参考平面退化（û ∥ z）、上臂与肩心-腕心连线共线（肘全伸 /
   *       全折），或结构不在闭式覆盖内（7 轴非 S-R-S 或上臂-前臂等长）；
   *       3) 6 轴无冗余自由度 → PARAM_ERR。
   * @param[in] joints 关节角 (deg，长度 = 轴数)
   * @param[out] arm_angle_rad 臂角 (rad)；未成功时不被改写
   * @return OK=成功；NO_SOLUTION=ψ 无定义；PARAM_ERR=6 轴调用 / 模型或入参非法
   */
  KinematicsStatus arm_angle_of(const Joints& joints, double& arm_angle_rad) const noexcept;

  /**
   * @brief 逆运动学全解：6 轴返回全部候选分支，7 轴求解目标位姿在给定臂角下的全部候选分支。
   * @note 1) 6 轴：写入全部存在的分支（≤ 8 支，form 编号定义同 ik）；指定臂角时被忽略——注意
   *       与本类 ik 有意不同：后者对 6 轴给有限臂角返回 PARAM_ERR；
   *       2) 7 轴：写入指定 ψ 下的 ≤ kMaxBranches 支（ψ 是硬约束）；默认 kArmAngleNone（NaN）代表
   *       未指定 → PARAM_ERR；
   *       3) 只写入限位内且收敛的解，按 form 升序，q 为规范代表；条数可小于 8（个别分支在该位姿
   *       不存在，如腕奇异时退化），不是错误；
   *       4) 失败时 out 为空；本接口不标 noexcept，需要零分配可先 out.reserve(kMaxBranches)。
   * @param[in] pose 目标位姿，6 元素 [X, Y, Z, A, B, C]（mm + rad）；工具手语义见 set_tool
   * @param[out] out 候选解集，按 form 升序；失败时为空
   * @param[in] arm_angle_rad 臂角 ψ (rad)，仅 7 轴有意义；默认 kArmAngleNone（NaN）表示未指定
   * @return OK=成功；SINGULAR=成功且至少一支处于腕奇异邻域；UNREACHABLE / NO_SOLUTION /
   * LIMIT_EXCEEDED / NOT_CONVERGED=求解失败；PARAM_ERR=入参 / 模型非法、或 7 轴未指定臂角
   */
  KinematicsStatus ik_all(const Poses& pose, std::vector<IKSolution>& out,
                          double arm_angle_rad = kArmAngleNone) const;

  /**
   * @brief ψ ∈ [lo, hi] 内按 step 采样并逐点求分支（离散采样，非全集）。仅 7 轴。
   * @note 1) 采样点为 ψ_j = lo + j·step，j = 0 .. floor((hi - lo) / step + 1e-12)，不额外补 hi（hi
   *       只界定范围）；出参 arm_angle_rad 原样回显采样值，lo / hi 超出 [-π, π) 时不折叠；
   *       2) 每个采样点的全部支（≤ 8）都写入，按 (ψ, form) 升序；q 为规范代表；
   *       3) 条数不可预知，失败时 out 为空；本接口不标 noexcept（std::vector 会分配）；
   *       4) 采样点数超过 kMaxSamplePoints 时返回 PARAM_ERR。
   * @param[in] pose 目标位姿，6 元素 [X, Y, Z, A, B, C]（mm + rad）；工具手语义见 set_tool
   * @param[out] out 采样解集，按 (ψ, form) 升序；失败时为空
   * @param[in] lo 下界 (rad)
   * @param[in] hi 上界 (rad)
   * @param[in] step 步长 (rad)，须 > 0
   * @return OK=成功；SINGULAR=成功且至少一支处于腕奇异邻域；UNREACHABLE / NO_SOLUTION /
   * LIMIT_EXCEEDED / NOT_CONVERGED=求解失败；PARAM_ERR=入参 / 模型非法、或 6 轴调用
   */
  KinematicsStatus ik_arm_angle_samples(const Poses& pose, std::vector<IKSolution>& out, double lo,
                                        double hi, double step) const;

  /**
   * @brief 限位约束下的 ψ 可行区间集合（仅 7 轴；避障主入口）。
   * @note 1) 一次返回全部 8 个 form 的区间，每条自带 form 标签、不跨 form 合并；
   *       2) 闭区间、按 (form, lo) 升序；同一 form 的多个可行段互不相邻（其间至少隔一个扫描样本的
   *       不可行带）；跨 ±π 的段拆成 [lo, π] 与 [-π, hi] 两条（端点原因 WRAP）；整圈可行（如未配置
   *       限位）时给出一条 lo = -π、hi = π 的段（两端 WRAP）；允许空结果（该位姿下所有 form 都无
   *       可行 ψ，不是错误）；
   *       3) 条数不可预知，失败时 out 为空；本接口不标 noexcept（std::vector 会分配）；
   *       4) 按 ψ 全周网格扫描 + 边界二分，窄于网格步长的可行 / 不可行段可能漏检；属规划 / 离线
   *       路径（非实时），耗时与缓存建议见《SDK 使用说明文档》「本地运动学接口」章节。
   * @param[in] pose 目标位姿，6 元素 [X, Y, Z, A, B, C]（mm + rad）；工具手语义见 set_tool
   * @param[out] out ψ 可行区间，按 (form, lo) 升序；失败时为空
   * @return OK=成功；UNREACHABLE=位置不可达；NO_SOLUTION=ψ 无定义；PARAM_ERR=入参 / 模型非法、
   * 或 6 轴调用
   */
  KinematicsStatus arm_angle_ranges(const Poses& pose, std::vector<ArmAngleRange>& out) const;

private:
  bool valid_links(const Links& links) noexcept;
  bool valid_limits(const Limits& limits) noexcept;

  /** 求解模型；解算过程只读 */
  KinematicsModel<DOF> model_{};
};

/**
 * @brief 由控制器参数构造本地运动学求解器（需要连接控制器）。
 * @note 1) 6 轴 / 7 轴均读控制器 DH（1~7 段，取前 DOF 段）与逐轴关节限位（J1~JDOF）后构造；
 *       2) 控制器机型轴数与 DOF 不符时返回 PARAM_ERR；
 *       3) 工具手为恒等变换，需要工具手时构造后调用 Kinematics::set_tool；
 *       4) 成功时整体覆盖 out（原对象的 tool 一并重置），失败时 out 不被改写；
 *       5) Python 侧同名入口 make_model(socketFd, axes)，axes 即本模板的 DOF。
 * @tparam DOF 轴数，取 6 或 7
 * @param[in] socket_fd 已连接的控制器套接字（6001 端口）
 * @param[out] out 构造出的求解器；成功时被整体覆盖，失败时不被改写
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 读取失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（机型不符、DH / 限位非法）；-4/-5/-6 原样透传
 * @warning 模型参数取自控制器回读值，控制器须已完成 DH 与关节限位标定；回读的 DH 出现退化段
 * （中间段 a、d 均为零且 alpha 为 0 或 180；末段 a、alpha、d 全零）时返回
 * PARAM_ERR，不上抛可用模型。
 * @attention 实验性接口：接口与行为可能随版本调整。
 */
template <int DOF>
TL_API Result make_model(SOCKETFD socket_fd, Kinematics<DOF>& out);

/** 显式实例化声明：实现由 SDK 库提供（6 / 7 轴），其他轴数无实现 */

extern template class Kinematics<6>;
extern template class Kinematics<7>;

/** make_model 的显式实例化声明（同理由 SDK 库提供）；extern template 声明处不得再加 TL_API
 *  （MSVC C4910） */

extern template Result make_model<6>(SOCKETFD socket_fd, Kinematics<6>& out);
extern template Result make_model<7>(SOCKETFD socket_fd, Kinematics<7>& out);

} // namespace tl

#endif /* TL_SDK_TL_KINEMATICS_H */
