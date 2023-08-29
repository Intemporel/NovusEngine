#include "Spline.h"

#include <utility>

#include "Base/Math/Interpolation.h"

namespace Spline
{
    SplinePath::SplinePath(SplineArguments arguments)
    {
        _arguments = arguments;
        _storage2D.Clear();
        _storage4D.Clear();
    }

    SplinePath::SplinePath(SplineArguments arguments, Spline2D spline)
    {
        _arguments = arguments;
        _spline2D = std::move(spline);
        _storage2D.Clear();
        _storage4D.Clear();
    }

    SplinePath::SplinePath(SplineArguments arguments, Spline4D spline)
    {
        _arguments = arguments;
        _spline4D = std::move(spline);
        _storage2D.Clear();
        _storage4D.Clear();
    }

    void SplinePath::SetSplineType(SplineType type)
    {
        _arguments.splineType = type;
        _storage2D.MarkAsDirty();
        _storage4D.MarkAsDirty();
    }

    void SplinePath::SetInterpolationType(InterpolationType type)
    {
        _arguments.interpolationType = type;
        _storage2D.MarkAsDirty();
        _storage4D.MarkAsDirty();
    }

    u32 SplinePath::GetSize()
    {
        if (Is2DSpline())
        {
            return static_cast<u32>(_spline2D.GetPoints().size());
        }
        else
        {
            return static_cast<u32>(_spline4D.GetPoints().size());
        }
    }

    void SplinePath::MarkAsDirty()
    {
        _storage2D.MarkAsDirty();
        _storage4D.MarkAsDirty();
    }

    template <typename T>
    void SplinePath::AddPoint(const DataSpline<T>& point, i32 position)
    {
        ControlSpline<T> control;

        if (Is2DSpline())
        {
            if (position < 0)
            {
                _spline2D.data.emplace_back(point);
                _spline2D.controls.emplace_back(control);
            }
            else
            {
                _spline2D.data.insert(_spline2D.data.begin() + position, point);
                _spline2D.controls.insert(_spline2D.controls.begin() + position, control);
            }

            _storage2D.MarkAsDirty();
        }
        else
        {
            if (position < 0)
            {
                _spline4D.data.emplace_back(point);
                _spline4D.controls.emplace_back(control);
            }
            else
            {
                _spline4D.data.insert(_spline4D.data.begin() + position, point);
                _spline4D.controls.insert(_spline4D.controls.begin() + position, control);
            }

            _storage4D.MarkAsDirty();
        }
    }

    void SplinePath::RemovePoint(u32 position)
    {
        if (Is2DSpline())
        {
            if (position >= _spline2D.data.size())
                return;

            auto dataIterator = _spline2D.data.begin();
            auto controlIterator = _spline2D.controls.begin();
            std::advance(dataIterator, position);
            std::advance(controlIterator, position);
            _spline2D.data.erase(dataIterator);
            _spline2D.controls.erase(controlIterator);
            _storage2D.MarkAsDirty();
        }
        else
        {
            if (position >= _spline4D.data.size())
                return;

            auto dataIterator = _spline4D.data.begin();
            auto controlIterator = _spline4D.controls.begin();
            std::advance(dataIterator, position);
            std::advance(controlIterator, position);
            _spline4D.data.erase(dataIterator);
            _spline4D.controls.erase(controlIterator);
            _storage4D.MarkAsDirty();
        }
    }

    void SplinePath::SetStep(u32 step)
    {
        step = Math::Max(static_cast<i32>(step), 1);

        if (_step == step)
            return;

        _step = step;
        _storage2D.SetStep(_step);
        _storage4D.SetStep(_step);
    }

    bool SplinePath::IsInterpolatedWithControl() const
    {
        switch (_arguments.interpolationType)
        {
            case InterpolationType::BSpline:
            case InterpolationType::CatmullRom:
            case InterpolationType::CatmullRom_Uniform:
            case InterpolationType::CatmullRom_Centripetal:
            case InterpolationType::CatmullRom_Chordal:
                return false;
            case InterpolationType::Hermite:
            case InterpolationType::Bezier:
            case InterpolationType::Linear:
            default:
                return true;
        }
    }

    bool SplinePath::Interpolate(bool force)
    {
        if (Is2DSpline())
        {
            if (!force && !_storage2D.IsDirty())
                return false;

            // a spline need at least to have 2 points minimum, otherwise it's just a point
            if (_spline2D.data.size() < 2 || _step == 0)
                return false;

            return Interpolate2D();
        }
        else
        {
            if (!force && !_storage4D.IsDirty())
                return false;

            // a spline need at least to have 2 points minimum, otherwise it's just a point
            if (_spline4D.data.size() < 2 || _step == 0)
                return false;

            return Interpolate4D();
        }
    }

    bool SplinePath::Interpolate2D()
    {
        _storage2D.Clear();
        f32 tStep = 1.0f / static_cast<f32>(_step);
        if (IsInterpolatedWithControl())
        {
            for (u32 i = 0; i < _spline2D.data.size() - 1; i++)
            {
                std::vector<f32> portion;
                portion.reserve(_step);

                for (u32 s = 0; s <= _step; s++)
                {
                    f32 t = tStep * static_cast<f32>(s);
                    portion.emplace_back(Interpolation2D(t, i));
                }

                _storage2D.AddPortion(portion);
            }
        }
        else
        {
            if (_spline2D.GetPoints().size() < 4)
                return false;

            for (u32 i = 0; i < _spline2D.GetPoints().size() - 4; i++)
            {
                std::vector<f32> portion;
                portion.resize(_step);

                for (u32 s = 0; s <= _step; s++)
                {
                    f32 t = tStep * static_cast<f32>(s);
                    portion.emplace_back(Interpolation2D(t, i));
                }

                _storage2D.AddPortion(portion);
            }
        }

        return true;
    }

    bool SplinePath::Interpolate4D()
    {
        _storage4D.Clear();
        f32 tStep = 1.0f / static_cast<f32>(_step);
        if (IsInterpolatedWithControl())
        {
            for (u32 i = 0; i < _spline4D.data.size() - 1; i++)
            {
                std::vector<vec3> portion;
                portion.reserve(_step);

                for (u32 s = 0; s <= _step; s++)
                {
                    f32 t = tStep * static_cast<f32>(s);
                    portion.emplace_back(Interpolation4D(t, i));
                }

                _storage4D.AddPortion(portion);
            }
        }
        else
        {
            if (_spline4D.GetPoints().size() < 4)
                return false;

            for (u32 i = 0; i < _spline4D.GetPoints().size() - 4; i++)
            {
                std::vector<vec3> portion;
                portion.resize(_step);

                for (u32 s = 0; s <= _step; s++)
                {
                    f32 t = tStep * static_cast<f32>(s);
                    portion.emplace_back(Interpolation4D(t, i));
                }

                _storage4D.AddPortion(portion);
            }
        }

        return true;
    }

    f32 SplinePath::Interpolation2D(f32 t, u32 index)
    {
        if (IsInterpolatedWithControl())
        {
            if (index >= _spline2D.data.size() - 1)
                return 0.0f;

            const auto& data0 = _spline2D.data[index];
            const auto& data1 = _spline2D.data[index + 1];
            const auto& control0 = _spline2D.controls[index];
            const auto& control1 = _spline2D.controls[index + 1];

            return InterpolationControl(t, data0, control0, data1, control1);
        }
        else
        {
            if (index >= _spline2D.data.size() - 4)
                return 0.0f;

            if (_spline2D.data.size() < 4)
                return 0.0f;

            return InterpolationCombined(t, _spline2D.GetPoints().data(), index, _spline2D.parameters);
        }
    }

    vec3 SplinePath::Interpolation4D(f32 t, u32 index)
    {
        if (IsInterpolatedWithControl())
        {
            if (index >= _spline4D.data.size() - 1)
                return vec3( 0.0f );

            const auto& data0 = _spline4D.data[index];
            const auto& data1 = _spline4D.data[index + 1];
            const auto& control0 = _spline4D.controls[index];
            const auto& control1 = _spline4D.controls[index + 1];

            return InterpolationControl(t, data0, control0, data1, control1);
        }
        else
        {
            if (index >= _spline4D.data.size() - 4)
                return vec3( 0.0f );

            if (_spline4D.data.size() < 4)
                return vec3( 0.0f );

            return InterpolationCombined(t, _spline4D.GetPoints().data(), index, _spline4D.parameters);
        }
    }

    template <typename T>
    T SplinePath::InterpolationControl(f32 t, const DataSpline<T>& data0, const ControlSpline<T>& control0, const DataSpline<T>& data1, const ControlSpline<T>& control1)
    {
        t = Math::Clamp(t, 0.0f, 1.0f);

        switch (_arguments.interpolationType)
        {
            case InterpolationType::Linear:
                return Interpolation::Linear::Lerp(t, data0.point, data1.point);
            case InterpolationType::Bezier:
                return Interpolation::Bezier::Cubic(t, data0.point, control0.out, control1.in, data1.point);
            case InterpolationType::Hermite:
                return Interpolation::Hermite::Cubic(t, data0.point, control0.out, control1.in, data1.point);
            default:
                return data0.point;
        }
    }

    template <typename T>
    T SplinePath::InterpolationCombined(f32 t, const T* points, const u32 index, SplineParameters parameters)
    {
        t = Math::Clamp(t, 0.0f, 1.0f);
        f32 alpha = Math::Clamp(parameters.param0, 0.0f, 1.0f);

        switch (_arguments.interpolationType)
        {
            case InterpolationType::BSpline:
                return Interpolation::BSpline::BasisSpline(t, points, index);
            case InterpolationType::CatmullRom:
                return Interpolation::CatmullRom::Base(t, alpha, points, index);
            case InterpolationType::CatmullRom_Uniform:
                return Interpolation::CatmullRom::Uniform(t, points, index);
            case InterpolationType::CatmullRom_Centripetal:
                return Interpolation::CatmullRom::Centripetal(t, points, index);
            case InterpolationType::CatmullRom_Chordal:
                return Interpolation::CatmullRom::Chordal(t, points, index);
            default:
                break;
        }

        T result = *(points + index);
        return result;
    }

    template <typename T>
    void SplinePath::UpdatePoint(u32 position, const DataSpline<T>& data, const ControlSpline<T>& control)
    {
        if (Is2DSpline())
        {
            if (position >= _spline2D.data.size())
                return;

            _spline2D.data[position] = data;
            _spline2D.controls[position] = control;
        }
        else
        {
            if (position >= _spline4D.data.size())
                return;

            _spline4D.data[position] = data;
            _spline4D.controls[position] = control;
        }
    }
}