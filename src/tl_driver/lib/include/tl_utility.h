/**
 * @file tl_utility.h
 * @brief 本地数学工具接口（角度/弧度换算、坐标与姿态变换，纯本地计算）
 * @note 约定：
 *       1) 四元数 4 元素 [w, x, y, z]（rpy2quat 输出 w >= 0）；欧拉角 3 元素 [rx, ry, rz]（弧度）
 *       2) 旋转序与控制器位姿一致：R = Rx(rx) * Ry(ry) * Rz(rz)，依次绕机体轴 X→Y→Z 旋转
 *       3) R 为 9 元素行主序；TR 为 16 元素行主序 [R|t; 0 0 0 1]
 *       4) 8 个变换接口（quat2rpy ~ tr2pose）的入参与返回值均为定长 std::array（3/4/6/9/16
 *          元素），元素个数编译期确定；deg2rad / rad2deg 另有标量（double）、任意长度
 *          std::array<double, N> 与 std::vector<double> 逐元素重载
 *       5) 输入不做有限性校验，NaN/Inf 按浮点语义传播，调用方需要时自行校验
 *       6) 控制器协议头文件（协议文档）注释中的形状（quat=3/rpy=4）与实际协议不符，勿采信
 *       7) 换算因子取自 M_PI（tl_types.h 跨平台兜底）：若在包含 SDK 头之前自定义 M_PI，
 *          将改变 deg2rad / rad2deg 的结果
 *       8) deg2rad 与 rad2deg 互为逆运算，但两次换算含浮点舍入，不保证逐位还原
 * @note 本文件为单头文件实现（header-only）：接口定义随 `tl_api.h` 一并包含，调用点可
 *       直接内联，不需要链接库导出符号。C++17 的 <cmath> 并非 constexpr：GCC 下经编译器
 *       内建（__builtin_*）实现常量折叠，可直接用于 static_assert 与 constexpr 变量；
 *       Clang/MSVC 无对应能力，相关接口退化为 inline（运行期求值，仍可内联）。纯算术
 *       接口（deg2rad / rad2deg、r2tr / tr2r）在所有编译器上均为 constexpr。std::vector
 *       重载在 C++17 下不可常量求值，保持 inline。
 * @attention 均为纯本地计算：不连接控制器、不发送报文。标量版与数组版不抛异常（noexcept）；
 * std::vector 版需分配内存，可能抛 std::bad_alloc。
 */
#ifndef TL_SDK_TL_UTILITY_H
#define TL_SDK_TL_UTILITY_H

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "tl_types.h"

namespace tl
{

// 换算因子：1° 对应的弧度数（具名常量，换算只求值一次；M_PI 由 tl_types.h 跨平台兜底）
constexpr double kRadPerDeg = M_PI / 180.0;

/**
 * @brief 角度转弧度。
 * @note 为纯算术 inline constexpr，可直接用于常量表达式（各编译器一致，不依赖数学内建）。
 * @param[in] deg 角度（度）
 * @return 弧度
 */
constexpr double deg2rad(double deg) noexcept
{
  return deg * kRadPerDeg;
}

/**
 * @brief 弧度转角度。
 * @note 为纯算术 inline constexpr；除以换算因子（而非乘 180/π），与 SDK 既有度/弧度归一
 * 在 IEEE754 double 构建下逐位一致。
 * @param[in] rad 弧度
 * @return 角度（度）
 */
constexpr double rad2deg(double rad) noexcept
{
  return rad / kRadPerDeg;
}

/**
 * @brief 数组逐元素角度转弧度。
 * @note 仅接受 std::array<double, N>（元素类型 double，不接受 float）；须显式构造 std::array
 * （列表初始化无法推导 N）。逐元素换算，仅适用于纯角度数组（如欧拉角、关节角）；
 * 混合单位的位姿不可整体换算（位移分量不应换算）。std::vector<double> 见下方同名重载。
 * 裸列表 {…}（2 个及以上元素）会解析到 std::vector 版、单个或空列表解析到标量版，请显式构造容器。
 * @tparam N 元素个数
 * @param[in] deg 角度数组（度）
 * @return 弧度数组
 */
template <std::size_t N>
constexpr std::array<double, N> deg2rad(const std::array<double, N>& deg) noexcept
{
  std::array<double, N> out{};
  for (std::size_t i = 0; i < N; ++i)
  {
    out[i] = deg2rad(deg[i]);
  }
  return out;
}

/**
 * @brief 数组逐元素弧度转角度。
 * @note 仅接受 std::array<double, N>（元素类型 double，不接受 float）；须显式构造 std::array
 * （列表初始化无法推导 N）。逐元素换算，仅适用于纯角度数组（如欧拉角、关节角）；
 * 混合单位的位姿不可整体换算（位移分量不应换算）。std::vector<double> 见下方同名重载。
 * 裸列表 {…}（2 个及以上元素）会解析到 std::vector 版、单个或空列表解析到标量版，请显式构造容器。
 * @tparam N 元素个数
 * @param[in] rad 弧度数组
 * @return 角度数组
 */
template <std::size_t N>
constexpr std::array<double, N> rad2deg(const std::array<double, N>& rad) noexcept
{
  std::array<double, N> out{};
  for (std::size_t i = 0; i < N; ++i)
  {
    out[i] = rad2deg(rad[i]);
  }
  return out;
}

/**
 * @brief 向量逐元素角度转弧度。
 * @note 返回新向量（一次堆分配）；同为逐元素换算，仅适用于纯角度向量（如关节角、欧拉角），
 * 混合单位的位姿不可整体换算（位移分量不应换算）。非 constexpr：C++17 的 std::vector 元素
 * 访问不是常量表达式。
 * @param[in] deg 角度向量（度）
 * @return 弧度向量
 */
inline std::vector<double> deg2rad(const std::vector<double>& deg)
{
  std::vector<double> out(deg.size());
  for (std::size_t i = 0; i < deg.size(); ++i)
  {
    out[i] = deg2rad(deg[i]);
  }
  return out;
}

/**
 * @brief 向量逐元素弧度转角度。
 * @note 返回新向量（一次堆分配）；同为逐元素换算，仅适用于纯角度向量（如关节角、欧拉角），
 * 混合单位的位姿不可整体换算（位移分量不应换算）。非 constexpr：C++17 的 std::vector 元素
 * 访问不是常量表达式。
 * @param[in] rad 弧度向量
 * @return 角度向量
 */
inline std::vector<double> rad2deg(const std::vector<double>& rad)
{
  std::vector<double> out(rad.size());
  for (std::size_t i = 0; i < rad.size(); ++i)
  {
    out[i] = rad2deg(rad[i]);
  }
  return out;
}

/**
 * @brief 本地数学工具的内部实现（不属于对外接口，勿直接调用）。
 * @note 全部 constexpr：GCC 下经数学内建实现常量折叠；其余编译器退化为 inline 运行期求值。
 */
namespace detail
{

// 编译期能力开关：GCC 的数学内建可被常量折叠（clang 的常量求值器不折叠这些内建，且
// __clang__ 同样定义 __GNUC__，故需显式排除）；其余编译器 <cmath> 在 C++17 亦非 constexpr，
// 且 MSVC 会以 C3615 拒绝「constexpr 函数中调用非 constexpr 函数」，故该类函数退化为 inline
// （内联不受影响，仅不能用于常量表达式）。
#if defined(__GNUC__) && !defined(__clang__)
#  define TL_UTILITY_CONSTEXPR constexpr
#  define TL_UTILITY_COS(x) __builtin_cos(x)
#  define TL_UTILITY_SIN(x) __builtin_sin(x)
#  define TL_UTILITY_ASIN(x) __builtin_asin(x)
#  define TL_UTILITY_ATAN2(y, x) __builtin_atan2(y, x)
#  define TL_UTILITY_SQRT(x) __builtin_sqrt(x)
#  define TL_UTILITY_FABS(x) __builtin_fabs(x)
#else
#  define TL_UTILITY_CONSTEXPR inline
#  define TL_UTILITY_COS(x) std::cos(x)
#  define TL_UTILITY_SIN(x) std::sin(x)
#  define TL_UTILITY_ASIN(x) std::asin(x)
#  define TL_UTILITY_ATAN2(y, x) std::atan2(y, x)
#  define TL_UTILITY_SQRT(x) std::sqrt(x)
#  define TL_UTILITY_FABS(x) std::fabs(x)
#endif

// ----定长载体（按值返回，拷贝消除后直接构造到调用方，不分配堆内存）----
using Rot3 = std::array<double, 9>;  // 旋转矩阵 3x3，行主序
using Rpy3 = std::array<double, 3>;  // 欧拉角 [rx, ry, rz]（弧度）
using Quat4 = std::array<double, 4>; // 四元数 [w, x, y, z]
using Tr4 = std::array<double, 16>;  // 齐次变换矩阵 4x4，行主序

// 万向节锁判定阈值：|ry| 距 π/2 在 1e-12 内视为精确锁死——四元数判据 |t2|→1
// 与矩阵判据 sy=|cos(ry)|→0 均在此阈值内触发奇异分支（rx/rz 的 atan2 分子分母为 0）
// 不写 inline：M_PI_2 允许被调用方覆盖（见 tl_types.h），外部链接会导致跨 TU 定义不一致
constexpr double kGimbalEps = 1e-12;
// π/2（万向节锁分支输出固定 pitch；M_PI_2 由 tl_types.h 跨平台兜底）
constexpr double kPiHalf = M_PI_2;

// ----标量原语（GCC 经内建可常量折叠；leaf 函数，标记 constexpr 便于内联/折叠）----
TL_UTILITY_CONSTEXPR double fabs_(double x) noexcept
{
  return TL_UTILITY_FABS(x);
}

TL_UTILITY_CONSTEXPR double cos_(double x) noexcept
{
  return TL_UTILITY_COS(x);
}

TL_UTILITY_CONSTEXPR double sin_(double x) noexcept
{
  return TL_UTILITY_SIN(x);
}

TL_UTILITY_CONSTEXPR double asin_(double x) noexcept
{
  return TL_UTILITY_ASIN(x);
}

TL_UTILITY_CONSTEXPR double sqrt_(double x) noexcept
{
  return TL_UTILITY_SQRT(x);
}

TL_UTILITY_CONSTEXPR double atan2_(double y, double x) noexcept
{
  return TL_UTILITY_ATAN2(y, x);
}

/**
 * @brief 数值钳位到 [lo, hi]。
 * @note 不用 std::min/std::max：本头文件会被包含 windows.h 的翻译单元引入，其 min/max
 * 宏会破坏 `std::min(` / `std::max(` 记号（MSVC C2589）。纯比较，各编译器均可常量求值。
 * @param[in] v 输入值
 * @param[in] lo 下限
 * @param[in] hi 上限
 * @return 钳位后的值
 */
constexpr double clamp_(double v, double lo, double hi) noexcept
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/**
 * @brief 欧拉角转旋转矩阵（R = Rx·Ry·Rz 绕机体轴依次 X→Y→Z，行主序 9 元素）。
 * @param[in] rx 绕机体 X 轴转角（弧度）
 * @param[in] ry 绕机体 Y 轴转角（弧度）
 * @param[in] rz 绕机体 Z 轴转角（弧度）
 * @return 旋转矩阵 9 元素，行主序
 */
TL_UTILITY_CONSTEXPR Rot3 rpy_to_r_matrix(double rx, double ry, double rz) noexcept
{
  const double cx = cos_(rx);
  const double sx = sin_(rx);
  const double cy = cos_(ry);
  const double sy = sin_(ry);
  const double cz = cos_(rz);
  const double sz = sin_(rz);
  return Rot3{{cy * cz,                // R[0][0]
               -cy * sz,               // R[0][1]
               sy,                     // R[0][2]
               sx * sy * cz + cx * sz, // R[1][0]
               cx * cz - sx * sy * sz, // R[1][1]
               -sx * cy,               // R[1][2]
               sx * sz - cx * sy * cz, // R[2][0]
               sx * cz + cx * sy * sz, // R[2][1]
               cx * cy}};              // R[2][2]
}

/**
 * @brief 四元数转欧拉角（与 rpy_to_r_matrix 同旋转序：R = Rx·Ry·Rz）。
 * @param[in] w 四元数标量分量
 * @param[in] x 四元数 X 分量
 * @param[in] y 四元数 Y 分量
 * @param[in] z 四元数 Z 分量
 * @return 欧拉角 3 元素 [rx, ry, rz]（弧度）
 */
TL_UTILITY_CONSTEXPR Rpy3 quat_to_rpy_angles(double w, double x, double y, double z) noexcept
{
  // t2 = sin(ry) = R[0][2]（四元数旋转矩阵元素 2(xz + wy)）
  const double t2 = clamp_(2.0 * (w * y + x * z), -1.0, 1.0);
  if (fabs_(fabs_(t2) - 1.0) < kGimbalEps)
  {
    // 精确万向节锁（|ry| = π/2）：rx/rz 的 atan2 分子分母同时为 0（结果由浮点噪声决定
    // 且不保持旋转），改由矩阵元素 R10/R11 确定 rx 并约定 rz = 0，分解保持旋转。
    // 与矩阵版 r_matrix_to_rpy_angles 奇异分支同约定：ry = -90° 时 R10 = -sin(rx)，
    // 需取负补偿
    const double r10 = 2.0 * (x * y + w * z);       // 四元数旋转矩阵 R[1][0]
    const double r11 = 1.0 - 2.0 * (x * x + z * z); // 四元数旋转矩阵 R[1][1]
    const double rx = (t2 > 0.0) ? atan2_(r10, r11) : -atan2_(r10, r11);
    return Rpy3{{rx, (t2 > 0.0) ? kPiHalf : -kPiHalf, 0.0}};
  }
  return Rpy3{{atan2_(2.0 * (w * x - y * z), 1.0 - 2.0 * (x * x + y * y)), // -R[1][2]/R[2][2]
               asin_(t2),
               atan2_(2.0 * (w * z - x * y), 1.0 - 2.0 * (y * y + z * z))}}; // -R[0][1]/R[0][0]
}

/**
 * @brief 欧拉角转四元数（与 rpy_to_r_matrix 同旋转序，q = qx ⊗ qy ⊗ qz）。
 * @param[in] rx 绕机体 X 轴转角（弧度）
 * @param[in] ry 绕机体 Y 轴转角（弧度）
 * @param[in] rz 绕机体 Z 轴转角（弧度）
 * @return 四元数 4 元素 [w, x, y, z]（w >= 0）
 */
TL_UTILITY_CONSTEXPR Quat4 rpy_to_quat_components(double rx, double ry, double rz) noexcept
{
  const double cr = cos_(rx * 0.5);
  const double sr = sin_(rx * 0.5);
  const double cp = cos_(ry * 0.5);
  const double sp = sin_(ry * 0.5);
  const double cy = cos_(rz * 0.5);
  const double sy = sin_(rz * 0.5);
  const double w = cr * cp * cy - sr * sp * sy;
  const double x = sr * cp * cy + cr * sp * sy;
  const double y = cr * sp * cy - sr * cp * sy;
  const double z = cr * cp * sy + sr * sp * cy;
  // 规范化 w >= 0：q 与 -q 表示同一旋转，统一取正 w 表示，
  // 保证输出 [x, y, z] 与 quat2rpy 用正根推导的 w 能还原同一旋转。
  return (w < 0.0) ? Quat4{{-w, -x, -y, -z}} : Quat4{{w, x, y, z}};
}

/**
 * @brief 从齐次变换矩阵提取左上 3x3 旋转矩阵（行主序）。
 * @note tr2r / tr2pose 共用：旋转块位于 0,1,2 / 4,5,6 / 8,9,10，平移列 3/7/11 不参与。
 * @param[in] tr 齐次变换矩阵 16 元素，行主序 [R|t; 0 0 0 1]
 * @return 旋转矩阵 9 元素，行主序
 */
constexpr Rot3 extract_r_from_tr(const Tr4& tr) noexcept
{
  return Rot3{{tr[0], tr[1], tr[2], tr[4], tr[5], tr[6], tr[8], tr[9], tr[10]}};
}

/**
 * @brief 旋转矩阵转欧拉角（R = Rx·Ry·Rz 提取，与 rpy_to_r_matrix 严格互逆）。
 * @note 万向节锁（|ry| = π/2）时约定 rz = 0、rx 由 R10/R11 确定，取负分支与
 * quat_to_rpy_angles 奇异分支同约定（分解保持旋转）。
 * @param[in] m 旋转矩阵 9 元素，行主序
 * @return 欧拉角 3 元素 [rx, ry, rz]（弧度）
 */
TL_UTILITY_CONSTEXPR Rpy3 r_matrix_to_rpy_angles(const Rot3& m) noexcept
{
  // cy = |cos(ry)|：行主序 R00=m[0]、R01=m[1]，cy² = R00² + R01²
  // （R = Rx·Ry·Rz 时 R00 = cy·cz、R01 = -cy·sz）
  const double cy = sqrt_(m[0] * m[0] + m[1] * m[1]);
  if (cy < kGimbalEps)
  {
    // 万向节锁（|ry| = π/2，cos(ry) ≈ 0）：以 m[2]=sin(ry) 判 ry 符号，
    // rx 由 R10/R11（m[3]/m[4]）确定；ry=-90° 时 R10 = -sin(rx)，取负补偿
    const double rx = (m[2] > 0.0) ? atan2_(m[3], m[4]) : -atan2_(m[3], m[4]);
    return Rpy3{{rx, (m[2] > 0.0) ? kPiHalf : -kPiHalf, 0.0}};
  }
  return Rpy3{{atan2_(-m[5], m[8]),   // -R12/R22
               atan2_(m[2], cy),      // R02/cy，数值稳定的 asin(R02) 等价式
               atan2_(-m[1], m[0])}}; // -R01/R00
}

} // namespace detail

/**
 * @brief 四元数转欧拉角。
 * @note 旋转序与控制器位姿一致（R = Rx·Ry·Rz）；GCC 下可常量求值。
 * @param[in] quat 四元数，4 元素 [w, x, y, z]（w 在首位；不校验单位性）
 * @return 欧拉角，3 元素 [rx, ry, rz]（弧度）
 */
TL_UTILITY_CONSTEXPR std::array<double, 3> quat2rpy(const std::array<double, 4>& quat) noexcept
{
  // 控制器协议：四元数 [w, x, y, z]，w 在首位
  return detail::quat_to_rpy_angles(quat[0], quat[1], quat[2], quat[3]);
}

/**
 * @brief 欧拉角转四元数。
 * @note 旋转序与控制器位姿一致（R = Rx·Ry·Rz）；GCC 下可常量求值。
 * @param[in] rpy 欧拉角，3 元素 [rx, ry, rz]（弧度）
 * @return 四元数，4 元素 [w, x, y, z]（w 在首位，w >= 0）
 */
TL_UTILITY_CONSTEXPR std::array<double, 4> rpy2quat(const std::array<double, 3>& rpy) noexcept
{
  // 控制器协议：四元数 [w, x, y, z]，w 在首位
  return detail::rpy_to_quat_components(rpy[0], rpy[1], rpy[2]);
}

/**
 * @brief 欧拉角转旋转矩阵。
 * @note 旋转序与控制器位姿一致：R = Rx(rx) * Ry(ry) * Rz(rz)；GCC 下可常量求值。
 * @param[in] rpy 欧拉角，3 元素 [rx, ry, rz]（弧度）
 * @return 旋转矩阵，9 元素，行主序
 */
TL_UTILITY_CONSTEXPR std::array<double, 9> rpy2r(const std::array<double, 3>& rpy) noexcept
{
  return detail::rpy_to_r_matrix(rpy[0], rpy[1], rpy[2]);
}

/**
 * @brief 旋转矩阵转欧拉角。
 * @note 旋转序与控制器位姿一致（R = Rx·Ry·Rz），与 rpy2r 互逆（|ry| = π/2 万向节锁处
 * 约定 rx/rz，分解保持旋转）；GCC 下可常量求值。
 * @param[in] r_matrix 旋转矩阵，9 元素，行主序
 * @return 欧拉角，3 元素 [rx, ry, rz]（弧度）
 */
TL_UTILITY_CONSTEXPR std::array<double, 3> r2rpy(const std::array<double, 9>& r_matrix) noexcept
{
  return detail::r_matrix_to_rpy_angles(r_matrix);
}

/**
 * @brief 旋转矩阵转位姿矩阵。
 * @note 平移分量置 0；纯算术，各编译器均可常量求值。
 * @param[in] r_matrix 旋转矩阵，9 元素，行主序
 * @return 位姿矩阵，16 元素，行主序 [R|0; 0 0 0 1]
 */
constexpr std::array<double, 16> r2tr(const std::array<double, 9>& r_matrix) noexcept
{
  return std::array<double, 16>{{r_matrix[0], r_matrix[1], r_matrix[2],
                                 0.0, // 平移 x
                                 r_matrix[3], r_matrix[4], r_matrix[5],
                                 0.0, // 平移 y
                                 r_matrix[6], r_matrix[7], r_matrix[8],
                                 0.0,                  // 平移 z
                                 0.0, 0.0, 0.0, 1.0}}; // 齐次右下角
}

/**
 * @brief 位姿矩阵转旋转矩阵。
 * @note 取位姿矩阵左上 3x3，平移分量不参与；纯取址，各编译器均可常量求值。
 * @param[in] tr_matrix 位姿矩阵，16 元素，行主序 [R|t; 0 0 0 1]
 * @return 旋转矩阵，9 元素，行主序
 */
constexpr std::array<double, 9> tr2r(const std::array<double, 16>& tr_matrix) noexcept
{
  return detail::extract_r_from_tr(tr_matrix); // tr2pose 亦复用同一提取
}

/**
 * @brief 位姿坐标转齐次变换矩阵。
 * @note 旋转序与控制器位姿一致：R = Rx(rx) * Ry(ry) * Rz(rz)；GCC 下可常量求值。
 * @param[in] pose 位姿坐标，6 元素 [x, y, z, rx, ry, rz]（位移 + 欧拉角，弧度）
 * @return 齐次变换矩阵，16 元素，行主序 [R|t; 0 0 0 1]
 */
TL_UTILITY_CONSTEXPR std::array<double, 16> pose2tr(const std::array<double, 6>& pose) noexcept
{
  const detail::Rot3 m = detail::rpy_to_r_matrix(pose[3], pose[4], pose[5]);
  return std::array<double, 16>{{m[0], m[1], m[2],
                                 pose[0], // 平移 x
                                 m[3], m[4], m[5],
                                 pose[1], // 平移 y
                                 m[6], m[7], m[8],
                                 pose[2],              // 平移 z
                                 0.0, 0.0, 0.0, 1.0}}; // 齐次右下角
}

/**
 * @brief 齐次变换矩阵转位姿坐标。
 * @note 旋转序与控制器位姿一致（R = Rx·Ry·Rz），与 pose2tr 互逆（万向节锁约定同 r2rpy）；
 * GCC 下可常量求值。
 * @param[in] tr_matrix 齐次变换矩阵，16 元素，行主序 [R|t; 0 0 0 1]
 * @return 位姿坐标，6 元素 [x, y, z, rx, ry, rz]（平移 + 欧拉角，弧度）
 */
TL_UTILITY_CONSTEXPR std::array<double, 6> tr2pose(const std::array<double, 16>& tr_matrix) noexcept
{
  const detail::Rpy3 rpy = detail::r_matrix_to_rpy_angles(detail::extract_r_from_tr(tr_matrix));
  return std::array<double, 6>{{tr_matrix[3], tr_matrix[7], tr_matrix[11], rpy[0], rpy[1], rpy[2]}};
}

} // namespace tl

// 内部开关不外泄（实现已在上方展开）
#undef TL_UTILITY_CONSTEXPR
#undef TL_UTILITY_COS
#undef TL_UTILITY_SIN
#undef TL_UTILITY_ASIN
#undef TL_UTILITY_ATAN2
#undef TL_UTILITY_SQRT
#undef TL_UTILITY_FABS

#endif /* TL_SDK_TL_UTILITY_H */
