#pragma once

#include "Base/Types.h"

namespace Spline
{
    struct SplineDataInformation
    {
        f32 min = std::numeric_limits<f32>::max();
        f32 max = std::numeric_limits<f32>::min();
        f32 average = 0.0f;
    };

    struct SplineInformation
    {
        SplineDataInformation distance;
        SplineDataInformation time;
    };

    template <typename T>
    class InterpolatedStorage
    {
    public:
        explicit InterpolatedStorage(u32 step)
            : _step(step)
        {
            Clear();
        }

        void SetStep(u32 step)
        {
            _step = step;
            Clear();
        }

        void SetInterpolation(const std::vector<T>& points, const std::vector<f32>& distance, const std::vector<f32>& time, u32 step)
        {
            _step = step;
            _points = points;
            _distances = distance;
            _times = time;
            _dirty = false;
        }

        void AddPortion(const std::vector<T>& portion, const std::vector<f32>& distance, const std::vector<f32>& time, i32 position = -1)
        {
            if (_step == 0 || (portion.size() != (_step + 1)) || portion.size() != distance.size())
                return;

            if (position < 0)
            {
                _points.insert(_points.end(), portion.begin(), portion.end());
                _distances.insert(_distances.end(), distance.begin(), distance.end());
                _times.insert(_times.end(), time.begin(), time.end());
                _dirty = false;
            }
            else
            {
                u32 newPosition = static_cast<u32>(position) * (_step + 1);
                if (position < _points.size())
                {
                    _points.insert(_points.begin() + newPosition, portion.begin(), portion.end());
                    _distances.insert(_distances.begin() + newPosition, distance.begin(), distance.end());
                    _times.insert(_times.begin() + newPosition, time.begin(), time.end());
                    _dirty = false;
                }
            }
        }

        SplineInformation GetInformation()
        {
            SplineInformation result;

            for (const auto& distance : _distances)
            {
                result.distance.min = std::min(result.distance.min, distance);
                result.distance.max = std::max(result.distance.max, distance);
                result.distance.average += distance;
            }
            result.distance.average /= static_cast<f32>(_distances.size());

            for (const auto& time : _times)
            {
                result.time.min = std::min(result.time.min, time);
                result.time.max = std::max(result.time.max, time);
                result.time.average += time;
            }
            result.time.average /= static_cast<f32>(_distances.size());

            return result;
        }

        void Clear() { _points.clear(); _distances.clear(); _times.clear(); MarkAsDirty(); }
        void MarkAsDirty() { _dirty = true; }

        [[nodiscard]] bool IsDirty() const { return _dirty; }
        const std::vector<T>& Storage() { return _points; };
        const std::vector<f32>& Distance() { return _distances; }
        const std::vector<f32>& Time() { return _times; }

    private:
        bool _dirty = true;
        u32 _step = 0;

        std::vector<T> _points = {};
        std::vector<f32> _distances = {};
        std::vector<f32> _times = {};
    };
}