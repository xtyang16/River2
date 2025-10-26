//
// Created by csl on 5/8/23.
//

#ifndef TINY_VIEWER_UTILS_H
#define TINY_VIEWER_UTILS_H

#include "pcl/visualization/pcl_visualizer.h"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/array.hpp"
#include "cereal/types/unordered_map.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/binary.hpp"
#include "cereal/types/polymorphic.hpp"
#include "fstream"
#include "pangolin/gl/colour.h"
#include "pangolin/gl/opengl_render_state.h"

namespace ns_viewer {    
    template<class Scalar, int M, int Options = 0>
    using Vector = Eigen::Matrix<Scalar, M, 1, Options>;

    template<class Scalar, int Options = 0>
    using Vector2 = Vector<Scalar, 2, Options>;
    using Vector2f = Vector2<float>;
    using Vector2d = Vector2<double>;

    template<class Scalar, int Options = 0>
    using Vector3 = Vector<Scalar, 3, Options>;
    using Vector3f = Vector3<float>;
    using Vector3d = Vector3<double>;

    template<class Scalar>
    using Vector4 = Vector<Scalar, 4>;
    using Vector4f = Vector4<float>;
    using Vector4d = Vector4<double>;

    template<class Scalar>
    using Vector6 = Vector<Scalar, 6>;
    using Vector6f = Vector6<float>;
    using Vector6d = Vector6<double>;

    template<class Scalar>
    using Vector7 = Vector<Scalar, 7>;
    using Vector7f = Vector7<float>;
    using Vector7d = Vector7<double>;

    template<class Scalar, int M, int N>
    using Matrix = Eigen::Matrix<Scalar, M, N>;

    template<class Scalar>
    using Matrix2 = Matrix<Scalar, 2, 2>;
    using Matrix2f = Matrix2<float>;
    using Matrix2d = Matrix2<double>;

    template<class Scalar>
    using Matrix3 = Matrix<Scalar, 3, 3>;
    using Matrix3f = Matrix3<float>;
    using Matrix3d = Matrix3<double>;

    template<class Scalar>
    using Matrix4 = Matrix<Scalar, 4, 4>;
    using Matrix4f = Matrix4<float>;
    using Matrix4d = Matrix4<double>;

    template<class Scalar>
    using Matrix6 = Matrix<Scalar, 6, 6>;
    using Matrix6f = Matrix6<float>;
    using Matrix6d = Matrix6<double>;

    template<class Scalar>
    using Matrix7 = Matrix<Scalar, 7, 7>;
    using Matrix7f = Matrix7<float>;
    using Matrix7d = Matrix7<double>;

    template<class Scalar>
    using Matrix34 = Matrix<Scalar, 3, 4>;
    using Matrix34f = Matrix34<float>;
    using Matrix34d = Matrix34<double>;

#define INVALID_TIME_STAMP (-1.0)

    static const float DEG_TO_RAD = M_PI / 180.0;

    static const float RAD_TO_DEG = 180.0 / M_PI;

#define DefaultLineSize (2.0f)
#define DefaultCoordSize (1.0f)
#define DefaultCubeSize (0.5f)
#define DefaultPointSize (4.0f)
#define DefaultIMUSize (0.2f)
#define DefaultCameraSize (0.2f)
#define DefaultRadarSize (0.2f)
#define DefaultLiDARSize (0.2f)
#define DefaultLandmarkSize (0.05f)

#define ExpandVec3(v) v(0), v(1), v(2)
#define ExpandStdVec3(v) v.at(0), v.at(1), v.at(2)
#define ExpandAryVec3(v) v[0], v[1], v[2]
#define ExpandPCLPointXYZ(p) p.x, p.y, p.z
#define ExpandColor(c) c.r, c.g, c.b, c.a
#define ExpandPCLColor(p) p.r * 0.00392, p.g * 0.00392, p.b * 0.00392, p.a * 0.00392

    using IntensityMode = pcl::visualization::LookUpTableRepresentationProperties;
    using ColourWheel = pangolin::ColourWheel;
    using Colour = pangolin::Colour;

    vtkSmartPointer <vtkLookupTable>
    GetColormapLUT(pcl::visualization::LookUpTableRepresentationProperties colormapType, double minmax[2]);

    std::pair<Eigen::Vector3f, Eigen::Vector3f> TangentBasis(const Eigen::Vector3f &v);

    std::optional<Eigen::Vector3f>
    LinePlaneIntersection(const Eigen::Vector3f &ls, const Eigen::Vector3f &le, const Eigen::Vector3f &norm, float d);

    std::vector<std::string> StringSplit(const std::string &str, char splitor, bool ignoreEmpty = true);
}

namespace Eigen {
    template<class Archive, typename ScaleType, int Rows, int Cols>
    void serialize(Archive &archive, Eigen::Matrix<ScaleType, Rows, Cols> &m) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                archive(cereal::make_nvp('r' + std::to_string(i) + 'c' + std::to_string(j), m(i, j)));
            }
        }
    }

    template<class Archive, typename ScaleType, int Cols>
    void serialize(Archive &archive, Eigen::Matrix<ScaleType, Eigen::Dynamic, Cols> &m) {
        for (int i = 0; i < m.rows(); ++i) {
            for (int j = 0; j < Cols; ++j) {
                archive(cereal::make_nvp('r' + std::to_string(i) + 'c' + std::to_string(j), m(i, j)));
            }
        }
    }

    template<class Archive, typename ScaleType>
    void serialize(Archive &archive, Eigen::Matrix<ScaleType, Eigen::Dynamic, Eigen::Dynamic> &m) {
        for (int i = 0; i < m.rows(); ++i) {
            for (int j = 0; j < m.cols(); ++j) {
                archive(cereal::make_nvp('r' + std::to_string(i) + 'c' + std::to_string(j), m(i, j)));
            }
        }
    }

    template<class Archive, typename ScaleType>
    void serialize(Archive &archive, Eigen::Quaternion<ScaleType> &q) {
        archive(
                cereal::make_nvp("qx", q.coeffs()[0]),
                cereal::make_nvp("qy", q.coeffs()[1]),
                cereal::make_nvp("qz", q.coeffs()[2]),
                cereal::make_nvp("qw", q.coeffs()[3])
        );
    }
}

namespace pangolin {
    template<class Archive>
    void serialize(Archive &ar, ns_viewer::Colour &color) {
        ar(
                cereal::make_nvp("red", color.red),
                cereal::make_nvp("green", color.green),
                cereal::make_nvp("blue", color.blue),
                cereal::make_nvp("alpha", color.alpha)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, OpenGlMatrix &m) {
        for (double &i: m.m) { ar(i); }
    }

    template<class Archive>
    void serialize(Archive &ar, OpenGlRenderState &m) {
        ar(
                cereal::make_nvp("projection_mat", m.GetProjectionMatrix()),
                cereal::make_nvp("model_view_mat", m.GetModelViewMatrix())
        );
    }
}

namespace pcl {
    template<class Archive>
    void serialize(Archive &ar, PCLHeader &h) {
        ar(
                cereal::make_nvp("stamp", h.stamp),
                cereal::make_nvp("frame_id", h.frame_id),
                cereal::make_nvp("seq", h.seq)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZI &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z),
                cereal::make_nvp("intensity", p.intensity)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZRGB &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z),
                cereal::make_nvp("r", p.r),
                cereal::make_nvp("g", p.g),
                cereal::make_nvp("b", p.b)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZRGBA &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z),
                cereal::make_nvp("r", p.r),
                cereal::make_nvp("g", p.g),
                cereal::make_nvp("b", p.b),
                cereal::make_nvp("a", p.a)
        );
    }

    template<class Archive>
    void serialize(Archive &ar, pcl::PointXYZ &p) {
        ar(
                cereal::make_nvp("x", p.x),
                cereal::make_nvp("y", p.y),
                cereal::make_nvp("z", p.z)
        );
    }

    template<class Archive, typename PointType>
    void serialize(Archive &ar, pcl::PointCloud<PointType> &cloud) {
        ar(
                cereal::make_nvp("header", cloud.header),
                cereal::make_nvp("points", cloud.points),
                cereal::make_nvp("width", cloud.width),
                cereal::make_nvp("height", cloud.height),
                cereal::make_nvp("is_dense", cloud.is_dense),
                cereal::make_nvp("sensor_orientation_", cloud.sensor_orientation_),
                cereal::make_nvp("sensor_origin_", cloud.sensor_origin_)
        );
    }
}

#endif //TINY_VIEWER_UTILS_H
