#pragma once
#include "InterpolatedStorage.h"

#include "FileFormat/Shared.h"
#include "FileFormat/Novus/FileHeader.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

class Bytebuffer;

namespace Model
{
    struct ComplexModel;
}

namespace Spline
{
    template <typename T>
    struct DataSpline
    {
    public:
        T point = { };
        f32 timestamp = 0.0f;
    };

    template <typename T>
    struct ControlSpline
    {
    public:
        T in  = { };
        T out = { };
    };

    struct SplineParameters
    {
    public:
        f32 param0 = 0.0f;
    };

    struct Spline4D
    {
    public:
        Spline4D() = default;
        void Clear()
        {
            data.clear();
            controls.clear();
        }

        std::vector<vec3> GetPoints()
        {
            std::vector<vec3> result;
            result.resize(data.size());
            for (auto& point : data)
            {
                result.push_back(point.point);
            }
            return result;
        }

        std::vector<DataSpline<vec3>> data { };
        std::vector<ControlSpline<vec3>> controls { };
        SplineParameters parameters { };
    };

    struct Spline2D
    {
    public:
        Spline2D() = default;
        void Clear()
        {
            data.clear();
            controls.clear();
        }

        std::vector<f32> GetPoints()
        {
            std::vector<f32> result;
            result.resize(data.size());
            for (auto& point : data)
            {
                result.push_back(point.point);
            }
            return result;
        }

        std::vector<DataSpline<f32>> data { };
        std::vector<ControlSpline<f32>> controls { };
        SplineParameters parameters { };
    };

    enum class InterpolationType
    {
        None,                       // (fake) control
        Linear,                     // (fake) control
        Bezier,                     // control
        Hermite,                    // control
        BSpline,                    // no control
        CatmullRom,                 // no control
        CatmullRom_Uniform,         // no control
        CatmullRom_Centripetal,     // no control
        CatmullRom_Chordal,         // no control
        COUNT
    };

    enum class SplineType
    {
        SplineType_2D = 0,
        SplineType_4D = 1
    };

    struct SplineArguments
    {
    public:
        SplineType splineType = SplineType::SplineType_2D;
        InterpolationType interpolationType = InterpolationType::None;
    };

    struct SplinePath
    {
    public:
        static constexpr u32 CURRENT_VERSION = 1;

        SplinePath();
        explicit SplinePath(SplineArguments parameters);
        SplinePath(SplineArguments parameters, Spline2D spline);
        SplinePath(SplineArguments parameters, Spline4D spline);

        void SetSplineType(SplineType type);
        void SetInterpolationType(InterpolationType type);
        [[nodiscard]] SplineType GetSplineType() const { return _arguments.splineType; }
        [[nodiscard]] InterpolationType GetInterpolationType() const { return _arguments.interpolationType; };
        [[nodiscard]] u32 GetSize();

        Spline2D& GetSpline2D() { return _spline2D; }
        Spline4D& GetSpline4D() { return _spline4D; }
        InterpolatedStorage<f32>& GetInterpolatedStorage2D() { return _storage2D; }
        InterpolatedStorage<vec3>& GetInterpolatedStorage4D() { return _storage4D; }
        void MarkAsDirty();

        [[nodiscard]] bool Is2DSpline() const { return _arguments.splineType == SplineType::SplineType_2D; }

        template <typename T>
        void AddPoint(const DataSpline<T>& point, i32 position = -1);
        void RemovePoint(u32 position);

        [[nodiscard]] const u32& Step() const { return _step; };
        void SetStep(u32 step);

        [[nodiscard]] bool IsInterpolatedWithControl() const;
        bool Interpolate(bool force = false);
        bool Interpolate2D();
        bool Interpolate4D();

        f32 Interpolation2D(f32 t, u32 index);
        vec3 Interpolation4D(f32 t, u32 index);
        template <typename T>
        T InterpolationControl(f32 t, const DataSpline<T>& data0, const ControlSpline<T>& control0, const DataSpline<T>& data1, const ControlSpline<T>& control1);
        template <typename T>
        T InterpolationCombined(f32 t, const T* points, u32 index, SplineParameters parameters);

        template <typename T>
        void UpdatePoint(u32 position, const DataSpline<T>& data, const ControlSpline<T>& control);

    protected:
        Spline2D _spline2D { };
        Spline4D _spline4D { };

        InterpolatedStorage<f32> _storage2D { 0 };
        InterpolatedStorage<vec3> _storage4D { 0 };

    private:
        SplineArguments _arguments;
        u32 _step = 1;

    public:
        FileHeader header = FileHeader(FileHeader::Type::Spline, CURRENT_VERSION);

    public:
        bool Save(const std::string& path);

        static bool Read(std::shared_ptr<Bytebuffer>& buffer, SplinePath& out);
        static bool FromComplexModel(const Model::ComplexModel& model, SplinePath& outPosition, SplinePath& outTarget, SplinePath& outRoll, SplinePath& outFov);
    };
}