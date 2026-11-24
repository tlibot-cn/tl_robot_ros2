/**
 * @file tl_robot_model.h
 * @brief 本地机器人模型接口（机器人基坐标系运动学正逆解，支持工具手变换与按控制器参数建模）。
 * @note 1) 关节角与关节限位一律为度 (deg)；位姿为 6 元素 [X, Y, Z, A, B, C]，位移 mm、姿态角
 *       rad，旋转序 R = Rx(A) * Ry(B) * Rz(C)。
 *       2) 只处理机器人基坐标系（BASE）位姿；工具手（TCP）变换见 RobotModel::set_tool，工具系 /
 *       用户系换算请用 convert_cartesian_coord / get_origin_coord_to_target_coord。
 *       3) 轴数在编译期固定：仅 6 轴与 7 轴两套实例（RobotModel<6> / RobotModel<7>）。
 *       4) 外部轴（协议点位第 7 位）不参与本地解算。
 *       5) 分支选解、臂角（ψ）与限位规则详见《SDK 使用说明文档》「本地运动学接口」章节。
 * @attention 除 make_model 外均为纯本地计算：不连接控制器、不发送报文；make_model 读取控制器参数
 * （不下发指令），需要已连接的控制器。
 */
#ifndef TL_SDK_TL_ROBOT_MODEL_H
#define TL_SDK_TL_ROBOT_MODEL_H

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
 * @note 1) 正逆解统一以本枚举为返回值，不用 Result 码；
 *       2) 成功状态：OK、SINGULAR_WRIST / SINGULAR_ELBOW / SINGULAR_SHOULDER /
 *       SINGULAR_ARM_ANGLE、BRANCH_SWITCHED，均写出出参；
 *       3) BRANCH_SWITCHED 在默认路径也可能返回；其余细化状态（SINGULAR_ELBOW /
 *       SINGULAR_SHOULDER / SINGULAR_ARM_ANGLE）仅在显式修改对应 IKOptions 字段时返回；
 *       4) NOT_CONVERGED 指数值精修未收敛；
 *       5) PARAM_ERR 表示入参 / 模型非法，此时正逆解都不改写出参。
 */
enum class KinematicsStatus
{
  OK = 0,             // 成功：解在限位内且非奇异邻域
  SINGULAR_WRIST = 1, // 成功：处于腕奇异邻域（6 轴 |sin(q5)| < 1e-6；7 轴 |sin(q6)| < 1e-6）
  SINGULAR_ELBOW = 2,    // 成功：肘奇异（解仍有效）
  SINGULAR_SHOULDER = 3, // 成功：肩奇异（解仍有效）
  SINGULAR_ARM_ANGLE = 4, // 成功：7 轴 ψ 无定义（参考平面退化 / 上臂共线），解仍有效
  BRANCH_SWITCHED = 5, // 成功：解有效但与参考构型不同支（仅 PREFER_FORM）
  UNREACHABLE = 6,     // 失败：目标位置超出可达范围（闭式分支不存在）
  NO_SOLUTION = 7,     // 失败：指定 form / ψ 的分支无解
  LIMIT_EXCEEDED = 8,  // 失败：存在几何解但全部候选越界
  NOT_CONVERGED = 9,   // 失败：数值精修未收敛
  PARAM_ERR = -1,      // 失败：入参 / 模型非法
};

/**
 * @brief 本地运动学求解模型：由 RobotModel<DOF> 持有。
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
  /** 是否合法（见 RobotModel::is_valid） */
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
 * @brief 分支策略（form = 0 时控制选支；所有情况下控制 7 轴相位 2 是否启用）。
 */
enum class BranchPolicy
{
  NEAREST = 0, // 候选集内按参考点距离选最近；允许 7 轴全周网格兜底
  PREFER_FORM, // 优先沿用参考 form；无同支解或同支解过远时改取更近候选（换支则返回
               // BRANCH_SWITCHED）；不做全周网格
  LOCK_FORM, // 只接受参考关节角的 form；同支无解返回底层失败状态，不写出、不换支；不做全周网格
};

/** 默认 ψ 全周兜底网格点数（也是当前实现的上限）。 */
constexpr int kDefaultPsiSamples = 16;

/**
 * @brief per-call 求解参数；默认构造 = 连续性优先（PREFER_FORM）。
 * @note 1) 默认路径可能返回 BRANCH_SWITCHED；SINGULAR_ELBOW / SINGULAR_SHOULDER /
 *       SINGULAR_ARM_ANGLE 仍需显式修改对应字段才返回；
 *       2) form != 0 时 branch_policy 不参与选支，但仍控制相位 2；
 *       3) arm_angle_rad 为有限值时是硬约束，6 轴返回 PARAM_ERR；
 *       4) 参考关节角的 form 无法唯一判定（处于分支重合位形）时，PREFER_FORM / LOCK_FORM
 *       退化为 NEAREST 选支，不返回 BRANCH_SWITCHED。
 */
struct IKOptions
{
  /** 形态：0 = 自动；1..8 = 指定分支 */
  int form{0};
  /** 臂角 ψ (rad)，仅 7 轴有效；NaN = 不指定 */
  double arm_angle_rad{kArmAngleNone};
  /** 分支策略：form = 0 时控制选支；所有情况下控制相位 2 是否启用 */
  BranchPolicy branch_policy{BranchPolicy::PREFER_FORM};
  /** 7 轴相位 2 全周网格点数；仅 NEAREST 生效；0 = 不做全周网格；上限 kDefaultPsiSamples */
  int psi_samples{kDefaultPsiSamples};
};

/**
 * @brief 单条候选解（6 / 7 轴的 ik_all、7 轴的 ik_arm_angle_samples 出参）。
 * @note 1) q 为规范代表：配置限位时取限位窗口内的等价表示，未配置限位时折到 (-π, π]；
 *       2) arm_angle_rad 在 6 轴无臂角概念，恒为 kArmAngleNone；
 *       3) status 只表达奇异类型；限位由 limit_joint 单独表达，二者可同时出现；
 *       4) 同一候选多个奇异条件同时满足时按优先级只取一个：腕 → 肘 → 肩。
 */
struct IKSolution
{
  std::array<double, 7> q{}; // 关节角 (deg，前 axes 项有效)
  int form{0};               // 形态编号（1..8，定义见 RobotModel::ik）
  int axes{0}; // q 的有效长度（0 = 未填充；出参恒为求解器轴数：6 轴 6，7 轴 7）
  double arm_angle_rad{kArmAngleNone};           // 该解对应的 ψ (rad)
  int limit_joint{-1};                           // -1 = 限位内；1..DOF = 首个越限关节号
  KinematicsStatus status{KinematicsStatus::OK}; // OK / SINGULAR_WRIST / SINGULAR_ELBOW /
                                                 // SINGULAR_SHOULDER / SINGULAR_ARM_ANGLE
};

/**
 * @brief 本地机器人模型（当前提供运动学求解；轴数编译期固定）。
 * @note 1) 仅 6 轴与 7 轴两套实例，其他轴数在编译期报错；
 *       2) 构造时接管连杆参数与限位；工具手由 set_tool 设置（默认恒等 = 无工具手）；
 *       3) 除 set_tool 外对象无可变状态：const 方法可并发调用、不分配堆内存、不抛异常（出参为
 *       std::vector 的入口除外）；set_tool 改写内部状态，不得与并发的解算调用同时进行。
 * @tparam DOF 轴数，取 6 或 7
 * @attention 构造不校验入参合法性（不抛异常），非法模型由 is_valid() 报告、调用时返回 PARAM_ERR。
 */
template <int DOF>
class RobotModel
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
  RobotModel() = default;

  /**
   * @brief 由连杆参数构造（不校验关节限位）。
   * @param[in] links 连杆参数（标准 DH，长度 = 轴数）
   */
  explicit RobotModel(const Links& links) noexcept;

  /**
   * @brief 由连杆参数与关节限位构造。
   * @param[in] links 连杆参数（标准 DH，长度 = 轴数）
   * @param[in] limits 关节限位 (deg，长度 = 轴数)
   */
  RobotModel(const Links& links, const Limits& limits) noexcept;

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
   * @brief 逆运动学解算（默认参数）：等价于传入默认构造的 IKOptions。
   * @note 1) form = 0：默认 branch_policy = PREFER_FORM，优先沿用 ref_joints 的 form；无同支解时
   *       换支并返回 BRANCH_SWITCHED；同支解与参考点距离超过 0.25 rad 时改取更近候选（可能
   *       同支），若换支则返回 BRANCH_SWITCHED（解仍写出）；
   *       2) 有效解返回 OK / SINGULAR_WRIST / BRANCH_SWITCHED，不返回其他细化状态；失败时返回对应
   *       失败状态；
   *       3) 7 轴不指定臂角时按「连续性优先」选解：先用参考关节角自身的臂角 ψ(ref) 求解；该 ψ
   *       无可行解、或其解不够连续（关节空间欧氏范数 > 0.25 rad，严格大于）时，在 ψ(ref) ± 12°
   *       内按 1.5° 步长精搜；默认不做全周网格兜底（需要时显式传 branch_policy = NEAREST）。
   * @param[in] pose 目标位姿，6 元素 [X, Y, Z, A, B, C]（mm + rad）；工具手语义见 set_tool
   * @param[in] ref_joints 参考关节角 (deg，长度 = 轴数)，用于自动选解与等效角归一；无默认值
   * @param[out] joints 关节角 (deg，长度 = 轴数)；未成功时不被改写
   * @return OK / SINGULAR_WRIST / BRANCH_SWITCHED=成功；其余取值为失败（区分见 KinematicsStatus）
   */
  KinematicsStatus ik(const Poses& pose, Joints& joints, const Joints& ref_joints) const noexcept;

  /**
   * @brief 逆运动学解算（显式求解参数）。
   * @note 1) opts.form ∈ [1,8] 为分支编号 4*肩 + 2*肘 + 腕 + 1（逐值含义见《SDK 使用说明文档》
   *       「本地运动学接口」），此时 branch_policy 不参与选支，但仍控制相位 2；
   *       2) opts.branch_policy = PREFER_FORM（默认）：优先沿用 ref_joints 的 form；无同支解时
   *       换支并返回 BRANCH_SWITCHED；同支解与参考点距离超过 0.25 rad 时改取更近候选（可能
   *       同支），若换支则返回 BRANCH_SWITCHED（解有效）；LOCK_FORM：同支无解时返回该次求解
   *       的失败状态（NO_SOLUTION / LIMIT_EXCEEDED / UNREACHABLE / NOT_CONVERGED），不写出、
   *       不换支；
   *       3) 默认路径可能返回 BRANCH_SWITCHED；SINGULAR_ELBOW / SINGULAR_SHOULDER /
   *       SINGULAR_ARM_ANGLE 仅在显式修改对应字段时返回，多个条件同时满足
   *       按优先级只返回一个：腕 → ψ 无定义（自由 ψ）→ 肘 → 肩 → BRANCH_SWITCHED → OK；
   *       4) opts.psi_samples 控制 7 轴末级全周网格点数，仅 branch_policy = NEAREST 生效
   *       （0 = 不做全周网格），取值 [0, kDefaultPsiSamples]；
   *       5) 6 轴指定有限 ψ 返回 PARAM_ERR；ref_joints 的 form 无法唯一判定（分支重合位形）时
   *       PREFER_FORM / LOCK_FORM 退化为 NEAREST 选支，仍不做相位 2；
   *       6) 本地逆解仅支持闭式结构（标准球腕 6 轴 / S-R-S 7 轴），结构不覆盖时返回
   *       NO_SOLUTION；「存在几何解但全部候选越限」返回 LIMIT_EXCEEDED，优先于 NO_SOLUTION。
   * @param[in] pose 目标位姿，6 元素 [X, Y, Z, A, B, C]（mm + rad）；工具手语义见 set_tool
   * @param[in] ref_joints 参考关节角 (deg，长度 = 轴数)，用于自动选解与等效角归一
   * @param[in] opts 求解参数；默认构造 = 默认重载行为
   * @param[out] joints 关节角 (deg，长度 = 轴数)；未成功时不被改写
   * @return OK / SINGULAR_* / BRANCH_SWITCHED=成功；其余取值为失败（区分见 KinematicsStatus）；
   * PARAM_ERR=入参 / 模型非法（含 form / ψ / psi_samples 越界）
   */
  KinematicsStatus ik(const Poses& pose, Joints& joints, const Joints& ref_joints,
                      const IKOptions& opts) const noexcept;

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
   *       3) 默认只写入限位内且收敛的解，按 form 升序，q 为规范代表；条数可小于 8（个别分支在该
   *       位姿不存在，如腕奇异时退化），不是错误；
   *       4) include_out_of_limit = true 时越限候选也写入：limit_joint 给出首个越限关节号
   *       （1..DOF），-1 表示限位内；status 表达该候选的奇异类型，与限位信息互不覆盖；
   *       5) 聚合返回值：只要有任一写入候选处于限位内，即按现有语义返回 OK / SINGULAR_WRIST；
   *       全部写入候选越限时返回 LIMIT_EXCEEDED（此时 out 仍非空，是诊断用途）；
   *       6) 失败时 out 为空；本接口不标 noexcept，需要零分配可先 out.reserve(kMaxBranches)。
   * @param[in] pose 目标位姿，6 元素 [X, Y, Z, A, B, C]（mm + rad）；工具手语义见 set_tool
   * @param[out] out 候选解集，按 form 升序；失败时为空
   * @param[in] arm_angle_rad 臂角 ψ (rad)，仅 7 轴有意义；默认 kArmAngleNone（NaN）表示未指定
   * @param[in] include_out_of_limit 是否写出越限候选（默认 false，保持原行为）
   * @return OK=成功；SINGULAR_WRIST=成功且至少一支处于腕奇异邻域；LIMIT_EXCEEDED=候选全部越限
   * 或不存在限位内解；UNREACHABLE / NO_SOLUTION / NOT_CONVERGED=求解失败；PARAM_ERR=入参 / 模型
   * 非法、或 7 轴未指定臂角
   */
  KinematicsStatus ik_all(const Poses& pose, std::vector<IKSolution>& out,
                          double arm_angle_rad = kArmAngleNone,
                          bool include_out_of_limit = false) const;

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
   * @return OK=成功；SINGULAR_WRIST=成功且至少一支处于腕奇异邻域；UNREACHABLE / NO_SOLUTION /
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
 *       3) 工具手为恒等变换，需要工具手时构造后调用 RobotModel::set_tool；
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
TL_API Result make_model(SOCKETFD socket_fd, RobotModel<DOF>& out);

/** 显式实例化声明：实现由 SDK 库提供（6 / 7 轴），其他轴数无实现 */

extern template class RobotModel<6>;
extern template class RobotModel<7>;

/** make_model 的显式实例化声明（同理由 SDK 库提供）；extern template 声明处不得再加 TL_API
 *  （MSVC C4910） */

extern template Result make_model<6>(SOCKETFD socket_fd, RobotModel<6>& out);
extern template Result make_model<7>(SOCKETFD socket_fd, RobotModel<7>& out);

} // namespace tl

#endif /* TL_SDK_TL_ROBOT_MODEL_H */
