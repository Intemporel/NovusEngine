#include "Spline.h"

#include <Base/Math/Interpolation.h>
#include <Base/Util/DebugHandler.h>

#include <fstream>
#include <utility>

namespace Spline
{
    SplinePath::SplinePath()
    {
        _storage2D.Clear();
        _storage4D.Clear();
    }

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

    /* Function used for Load and Save a .spline file */
    bool SplinePath::Save(const std::string& path)
    {
        std::ofstream output(path, std::ofstream::out | std::ofstream::binary);
        if (!output)
        {
            DebugHandler::PrintError("Failed to create Spline file. Check admin permissions");
            return false;
        }

        // Write Header
        {
            output.write(reinterpret_cast<const char*>(&header), sizeof(FileHeader));
        }

        // Write Arguments
        {
            output.write(reinterpret_cast<const char*>(&_arguments), sizeof(Spline::SplineArguments));
        }

        // Write Spline Data
        {
            if (Is2DSpline())
            {
                // Data
                auto dataSize = static_cast<u32>(_spline2D.data.size());
                output.write(reinterpret_cast<const char*>(&dataSize), sizeof(u32));

                if (dataSize > 0)
                {
                    output.write(reinterpret_cast<const char*>(&_spline2D.data[0]), dataSize * (2u * sizeof(f32)));
                }

                // Control
                auto controlSize = static_cast<u32>(_spline2D.controls.size());
                output.write(reinterpret_cast<const char*>(&controlSize), sizeof(u32));

                if (controlSize > 0)
                {
                    output.write(reinterpret_cast<const char*>(&_spline2D.controls[0]), dataSize * (2u * sizeof(f32)));
                }

                // Parameters
                output.write(reinterpret_cast<const char*>(&_spline2D.parameters), sizeof(Spline::SplineParameters));
            }
            else
            {
                // Data
                auto dataSize = static_cast<u32>(_spline4D.data.size());
                output.write(reinterpret_cast<const char*>(&dataSize), sizeof(u32));

                if (dataSize > 0)
                {
                    output.write(reinterpret_cast<const char*>(&_spline4D.data[0]), dataSize * (sizeof(vec3) + sizeof(f32)));
                }

                // Control
                auto controlSize = static_cast<u32>(_spline4D.controls.size());
                output.write(reinterpret_cast<const char*>(&controlSize), sizeof(u32));

                if (controlSize > 0)
                {
                    output.write(reinterpret_cast<const char*>(&_spline4D.controls[0]), dataSize * (2u * sizeof(vec3)));
                }

                // Parameters
                output.write(reinterpret_cast<const char*>(&_spline4D.parameters), sizeof(Spline::SplineParameters));
            }
        }

        return true;
    }

    bool SplinePath::Read(std::shared_ptr<Bytebuffer>& buffer, Spline::SplinePath& out)
    {

        return true;
    }

    bool SplinePath::FromComplexModel(const Model::ComplexModel& model, Spline::SplinePath& outPosition, Spline::SplinePath& outTarget, Spline::SplinePath& outRoll, Spline::SplinePath& outFov)
    {
        if (model.cameras.empty())
            return false;

        const auto& camera = model.cameras[0];
        if (camera.positions.tracks.empty() ||
            camera.targetPositions.tracks.empty() ||
            camera.roll.tracks.empty() ||
            camera.fov.tracks.empty())
        {
            return false;
        }

        outPosition.SetSplineType(Spline::SplineType::SplineType_4D);
        outTarget.SetSplineType(Spline::SplineType::SplineType_4D);
        outRoll.SetSplineType(Spline::SplineType::SplineType_2D);
        outFov.SetSplineType(Spline::SplineType::SplineType_2D);

        outPosition.SetInterpolationType(InterpolationTypeAnimToSpline(camera.positions.interpolationType));
        outTarget.SetInterpolationType(InterpolationTypeAnimToSpline(camera.targetPositions.interpolationType));
        outRoll.SetInterpolationType(InterpolationTypeAnimToSpline(camera.roll.interpolationType));
        outFov.SetInterpolationType(InterpolationTypeAnimToSpline(camera.fov.interpolationType));

        const Model::ComplexModel::AnimationTrack<SplineKey<vec3>>& positionTrack = camera.positions.tracks[0];
        const Model::ComplexModel::AnimationTrack<SplineKey<vec3>>& targetTrack = camera.targetPositions.tracks[0];
        const Model::ComplexModel::AnimationTrack<SplineKey<f32>>& rollTrack = camera.roll.tracks[0];
        const Model::ComplexModel::AnimationTrack<SplineKey<f32>>& fovTrack = camera.fov.tracks[0];

        vec3 positionBase = camera.positionBase;
        vec3 targetBase = camera.targetPositionBase;

        if ((positionTrack.values.size() != positionTrack.timestamps.size() && !positionTrack.values.empty()) ||
            (targetTrack.values.size() != targetTrack.timestamps.size() && !targetTrack.values.empty()) ||
            (rollTrack.values.size() != rollTrack.timestamps.size() && !rollTrack.values.empty()) ||
            (fovTrack.values.size() != fovTrack.timestamps.size() && !fovTrack.values.empty()))
        {
            return false;
        }

        f32 maxTimestamp = static_cast<f32>(positionTrack.timestamps[positionTrack.timestamps.size() - 1]);

        outPosition._spline4D.Clear();
        u32 numPositionValues = static_cast<u32>(positionTrack.values.size());
        for (u32 i = 0; i < numPositionValues; i++)
        {
            auto& point = outPosition._spline4D.data.emplace_back();
            point.point = (positionTrack.values[i].value + positionBase);
            point.timestamp = static_cast<f32>(positionTrack.timestamps[i]) / maxTimestamp;

            auto& control = outPosition._spline4D.controls.emplace_back();
            control.in = (positionTrack.values[i].tanIn + positionBase);
            control.out = (positionTrack.values[i].tanOut + positionBase);
        }

        outTarget._spline4D.Clear();
        u32 numTargetValues = static_cast<u32>(targetTrack.values.size());
        for (u32 i = 0; i < numTargetValues; i++)
        {
            auto& point = outTarget._spline4D.data.emplace_back();
            point.point = (targetTrack.values[i].value + targetBase);
            point.timestamp = static_cast<f32>(targetTrack.timestamps[i]) / maxTimestamp;

            auto& control = outTarget._spline4D.controls.emplace_back();
            control.in = (targetTrack.values[i].tanIn + targetBase);
            control.out = (targetTrack.values[i].tanOut + targetBase);
        }

        outRoll._spline2D.Clear();
        u32 numRollValues = static_cast<u32>(rollTrack.values.size());
        for (u32 i = 0; i < numRollValues; i++)
        {
            auto& point = outRoll._spline2D.data.emplace_back();
            point.point = rollTrack.values[i].value;
            point.timestamp = static_cast<f32>(rollTrack.timestamps[i]) / maxTimestamp;

            auto& control = outRoll._spline2D.controls.emplace_back();
            control.in = rollTrack.values[i].tanIn;
            control.out = rollTrack.values[i].tanOut;
        }

        outFov._spline2D.Clear();
        u32 numFovValues = static_cast<u32>(fovTrack.values.size());
        for (u32 i = 0; i < numFovValues; i++)
        {
            auto& point = outFov._spline2D.data.emplace_back();
            point.point = fovTrack.values[i].value;
            point.timestamp = static_cast<f32>(fovTrack.timestamps[i]) / maxTimestamp;

            auto& control = outFov._spline2D.controls.emplace_back();
            control.in = fovTrack.values[i].tanIn;
            control.out = fovTrack.values[i].tanOut;
        }

        return true;
    }

    Spline::InterpolationType SplinePath::InterpolationTypeAnimToSpline(Model::ComplexModel::AnimationInterpolationType type)
    {
        switch (type)
        {
            case Model::ComplexModel::AnimationInterpolationType::NONE:
                return Spline::InterpolationType::None;
            case Model::ComplexModel::AnimationInterpolationType::LINEAR:
                return Spline::InterpolationType::Linear;
            case Model::ComplexModel::AnimationInterpolationType::CUBIC_BEZIER_SPLINE:
                return Spline::InterpolationType::Bezier;
            case Model::ComplexModel::AnimationInterpolationType::CUBIC_HERMIT_SPLINE:
                return Spline::InterpolationType::Hermite;
        }

        return Spline::InterpolationType::None;
    }
}