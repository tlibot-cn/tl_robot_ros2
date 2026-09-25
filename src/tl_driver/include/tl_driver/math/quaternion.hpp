#ifndef TL_DRIVER__MATH__QUATERNION_HPP_
#define TL_DRIVER__MATH__QUATERNION_HPP_

#include <vector>

namespace tl_driver::math
{

/**
 * @brief 欧拉角(RPY, 内旋 XYZ, rad) 转四元数。
 * @param[in] rpy 欧拉角 [rx, ry, rz]，单位 rad
 * @return 四元数 [w, x, y, z]
 */
std::vector<double> rpy_to_quat(const std::vector<double>& rpy);

/**
 * @brief 四元数转欧拉角(RPY, 内旋 XYZ, rad)。
 * @param[in] q 四元数 [w, x, y, z]
 * @return 欧拉角 [rx, ry, rz]，单位 rad
 */
std::vector<double> quat_to_rpy(const std::vector<double>& q);

/**
 * @brief 四元数球面线性插值。
 * @param[in] q1 起始四元数 [w, x, y, z]
 * @param[in] q2 目标四元数 [w, x, y, z]
 * @param[in] t 插值系数，取值 [0, 1]
 * @return 插值后的四元数 [w, x, y, z]
 */
std::vector<double> quat_slerp(const std::vector<double>& q1, const std::vector<double>& q2, double t);

} // namespace tl_driver::math

#endif // TL_DRIVER__MATH__QUATERNION_HPP_
