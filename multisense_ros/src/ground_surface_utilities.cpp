/**
 * @file ground_surface_utilities.cpp
 *
 * Copyright 2021
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <multisense_ros/ground_surface_utilities.h>

namespace ground_surface_utilities {

namespace {

float computeAzimuth(const float a, const float b)
{
    return a >= 0 ? static_cast<float>(M_PI) - atan(b / (a + std::numeric_limits<float>::epsilon()))
                  : atan(b / -(a + std::numeric_limits<float>::epsilon()));
}

float computeRange(const float a, const float b)
{
    return sqrt(pow(a, 2) + pow(b, 2));
}

template <typename T> T computeQuadraticSurface(T x, T z, const T* params)
{
    ///
    /// Compute height of quadratic surface model
    ///     y = ax^2 + bz^2 + cxz + dx + ez + f
    ///
    return params[0] * pow(x, 2) + params[1] * pow(z, 2) + params[2] * x * z +
           params[3] * x + params[4] * z + params[5];
}

Eigen::Matrix<Eigen::Matrix<float, Eigen::Dynamic, 1>, Eigen::Dynamic, 1> generateBasisArray()
{
    Eigen::Matrix<Eigen::Matrix<float, Eigen::Dynamic, 1>, Eigen::Dynamic, 1> ret(4);

    // Temporary storage for polynomial coefficients
    Eigen::Matrix<float, Eigen::Dynamic, 1> basisCoefficients(4);

    // The following basis functions are copied from Lee, Wolberg, and Shin,
    // "Scattered Data Interpolation with Multilevel B-Splines",
    // IEEE Transactions on Visualization and Computer Graphics, Vol 3, 228-244, 1997.

    // First cubic spline basis component is
    // B(s) = (1 - s)**3 / 6.
    // This expands to
    // B(s) = -(1/6)s**3 + (1/2)s**2 - (1/2)s + 1/6.
    basisCoefficients(0) = 1.0 / 6.0;
    basisCoefficients(1) = -0.5;
    basisCoefficients(2) = 0.5;
    basisCoefficients(3) = -1.0 / 6.0;
    ret(0) = basisCoefficients;

    // Second cubic spline basis component is
    // B(s) = (1/2)t**3 - t**2 + 2/3.
    basisCoefficients(0) = 2.0 / 3.0;
    basisCoefficients(1) = 0.0;
    basisCoefficients(2) = -1.0;
    basisCoefficients(3) = 0.5;
    ret(1) = basisCoefficients;

    // Third cubic spline basis component is
    // B(s) = -(1/2)t**3 + (1/2)t**2 + (1/2)t + 1/6.
    basisCoefficients(0) = 1.0 / 6.0;
    basisCoefficients(1) = 0.5;
    basisCoefficients(2) = 0.5;
    basisCoefficients(3) = -0.5;
    ret(2) = basisCoefficients;

    // Fourth cubic spline basis component is
    // B(s) = (1/6)t**3.
    basisCoefficients(0) = 0.0;
    basisCoefficients(1) = 0.0;
    basisCoefficients(2) = 0.0;
    basisCoefficients(3) = 1.0 / 6.0;
    ret(3) = basisCoefficients;

    return ret;
}

template <class FloatT>
FloatT getSplineValue(
    const FloatT* xzCellOrigin,
    const FloatT* xzCellSize,
    const FloatT sValue,
    const FloatT tValue,
    const Eigen::Matrix<FloatT, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &controlGrid,
    const Eigen::Matrix<Eigen::Matrix<FloatT, Eigen::Dynamic, 1>, Eigen::Dynamic, 1> &basisArray)
{
    // Find the integer coords of the control grid cell in which the input point lies.
    FloatT iTmp = (sValue - xzCellOrigin[0]) / xzCellSize[0];
    FloatT jTmp = (tValue - xzCellOrigin[1]) / xzCellSize[1];

    int iCoord = static_cast<int>(floor(iTmp));
    int jCoord = static_cast<int>(floor(jTmp));

    // Find real valued coords within the cell, along with all of the powers of those coords
    // we'll be wanting to plug into spline basis functions.
    FloatT powersOfS[4];
    powersOfS[0] = FloatT(1.0);
    powersOfS[1] = iTmp - FloatT(iCoord);
    powersOfS[2] = powersOfS[1] * powersOfS[1];
    powersOfS[3] = powersOfS[2] * powersOfS[1];
    FloatT powersOfT[4];
    powersOfT[0] = FloatT(1.0);
    powersOfT[1] = jTmp - FloatT(jCoord);
    powersOfT[2] = powersOfT[1] * powersOfT[1];
    powersOfT[3] = powersOfT[2] * powersOfT[1];

    size_t iIndex = static_cast<size_t>(iCoord);
    size_t jIndex = static_cast<size_t>(jCoord);

    // Interpolate by adding spline basis functions from the surrounding control points.
    int index0 = iIndex - 1;
    int index1 = jIndex - 1;
    FloatT functionValue = 0.0;
    for (size_t kIndex = 0; kIndex < 4; ++kIndex)
    {
        size_t i0PlusK = index0 + kIndex;

        FloatT B_k = std::inner_product(
            powersOfS, powersOfS + 4, (basisArray)[kIndex].data(), static_cast<FloatT>(0));

        for (size_t lIndex = 0; lIndex < 4; ++lIndex)
        {
            size_t i1PlusL = index1 + lIndex;

            FloatT B_l = std::inner_product(
                powersOfT, powersOfT + 4, (basisArray)[lIndex].data(), static_cast<FloatT>(0));

            functionValue += (B_k * B_l * controlGrid(i1PlusL, i0PlusK));
        }
    }
    return functionValue;
}

} // anonymous namespace

Eigen::Matrix<uint8_t, 3, 1> groundSurfaceClassToPixelColor(const uint8_t value)
{
    if (value == 1) // Out of bounds
        return Eigen::Matrix<uint8_t, 3, 1>{ 114, 159, 207 };
    else if (value == 2) // Obstacle
        return Eigen::Matrix<uint8_t, 3, 1>{ 255, 0, 0 };
    else if (value == 3) // Free space
        return Eigen::Matrix<uint8_t, 3, 1>{ 0, 255, 0 };

    // Unknown
    return Eigen::Matrix<uint8_t, 3, 1>{ 0, 0, 0 };
}

sensor_msgs::PointCloud2 eigenToPointcloud(
    const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &input,
    const std::string &frame_id)
{
    sensor_msgs::PointCloud2 ret =
        multisense_ros::initializePointcloud<float, uint8_t>(true, frame_id, "intensity");

    const double num_points = input.size();
    ret.data.resize(num_points * ret.point_step);

    for (size_t i = 0; i < num_points; ++i)
    {
        float* cloudP = reinterpret_cast<float*>(&(ret.data[i * ret.point_step]));
        cloudP[0] = input[i][0];
        cloudP[1] = input[i][1];
        cloudP[2] = input[i][2];

        uint8_t* colorP = reinterpret_cast<uint8_t*>(&(cloudP[3]));
        colorP[0] = 0;
    }

    ret.height = 1;
    ret.row_step = num_points * ret.point_step;
    ret.width = num_points;
    ret.data.resize(num_points * ret.point_step);

    return ret;
}

std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> convertSplineToPointcloud(
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &controlGrid,
    const SplineDrawParameters &splineDrawParams,
    const double pointcloudMaxRange,
    const float* xzCellOrigin,
    const float* xzCellSize,
    const float* minMaxAzimuthAngle,
    const float* extrinsics,
    const float* quadraticParams,
    const float baseline)
{
    static const auto basisArray = generateBasisArray();

    // Generate extrinsics matrix
    Eigen::Matrix<float, 4, 4> extrinsicsMat;
    {
        extrinsicsMat.setZero();
        extrinsicsMat(0, 3) = extrinsics[0];
        extrinsicsMat(1, 3) = extrinsics[1];
        extrinsicsMat(2, 3) = extrinsics[2];
        extrinsicsMat(3, 3) = static_cast<float>(1.0);
        Eigen::Matrix<float, 3, 3> rot =
            (Eigen::AngleAxis<float>(extrinsics[5], Eigen::Matrix<float, 3, 1>(0, 0, 1))
            * Eigen::AngleAxis<float>(extrinsics[4], Eigen::Matrix<float, 3, 1>(0, 1, 0))
            * Eigen::AngleAxis<float>(extrinsics[3], Eigen::Matrix<float, 3, 1>(1, 0, 0))).matrix();
        extrinsicsMat.block(0, 0, 3, 3) = rot;
    }

    // Precompute extrinsics inverse
    const auto extrinsicsInverse = extrinsicsMat.inverse();

    // Precompute number of points that will be drawn
    const size_t numPoints =
        std::floor((splineDrawParams.max_x_m - splineDrawParams.min_x_m) / splineDrawParams.resolution) *
        std::floor((splineDrawParams.max_z_m - splineDrawParams.min_z_m) / splineDrawParams.resolution);

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points;
    points.reserve(numPoints);

    for (float x = splineDrawParams.min_x_m; x < splineDrawParams.max_x_m; x += splineDrawParams.resolution)
    {
        for (float z = splineDrawParams.min_z_m; z < splineDrawParams.max_z_m; z += splineDrawParams.resolution)
        {
            // Compute spline point and transform into left camera optical frame
            const auto y = getSplineValue(xzCellOrigin, xzCellSize, x, z, controlGrid, basisArray)
                           + computeQuadraticSurface(x, z, quadraticParams);

            const Eigen::Vector3f splinePoint = Eigen::Vector3f(x, y, z);
            const Eigen::Vector3f transformedSplinePoint = (extrinsicsInverse * splinePoint.homogeneous()).hnormalized();

            // Filter points by range and angle
            const auto distance = computeRange(transformedSplinePoint(0), transformedSplinePoint(2));
            if (distance > pointcloudMaxRange)
                continue;

            const auto leftCamAzimuthAngle = computeAzimuth(transformedSplinePoint(0), transformedSplinePoint(2));
            if (leftCamAzimuthAngle < minMaxAzimuthAngle[0])
                continue;

            // Offset max azimuth angle check by baseline for a cleaner "frustum" visualization
            const auto rightCamAzimuthAngle = computeAzimuth(transformedSplinePoint(0) + baseline, transformedSplinePoint(2));
            if (rightCamAzimuthAngle > minMaxAzimuthAngle[1])
                continue;

            points.emplace_back(splinePoint);
        }
    }

    points.shrink_to_fit();

    return points;
}

} // namespace ground_surface_utilities
