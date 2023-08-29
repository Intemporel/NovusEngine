#include "InterpolatedStorage.h"

namespace Spline
{
    template <typename T>
    InterpolatedStorage<T>::InterpolatedStorage(u32 step)
        : _step(step)
    {
        Clear();
    }

    template <typename T>
    void InterpolatedStorage<T>::SetStep(u32 step)
    {
        _step = step;
        Clear();
    }

    template <typename T>
    void InterpolatedStorage<T>::SetInterpolation(const std::vector<T>& points, u32 step)
    {
        _step = step;
        _points = points;
        _dirty = false;
    }

    template <typename T>
    void InterpolatedStorage<T>::AddPortion(const std::vector<T>& portion, i32 position)
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
}