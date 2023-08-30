#pragma once

#include "Base/Types.h"

namespace Spline
{
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

        void SetInterpolation(const std::vector<T>& points, u32 step)
        {
            _step = step;
            _points = points;
            _dirty = false;
        }

        void AddPortion(const std::vector<T>& portion, i32 position = -1)
        {
            if (_step == 0 || (portion.size() != (_step + 1)))
                return;

            if (position < 0)
            {
                _points.insert(_points.end(), portion.begin(), portion.end());
                _dirty = false;
            }
            else
            {
                u32 newPosition = static_cast<u32>(position) * (_step + 1);
                if (position < _points.size())
                {
                    _points.insert(_points.begin() + newPosition, portion.begin(), portion.end());
                    _dirty = false;
                }
            }
        }

        void Clear() { _points.clear(); MarkAsDirty(); }
        void MarkAsDirty() { _dirty = true; }

        [[nodiscard]] bool IsDirty() const { return _dirty; }
        const std::vector<T>& Storage() { return _points; };

    private:
        bool _dirty = true;
        u32 _step = 0;
        std::vector<T> _points = {};
    };
}