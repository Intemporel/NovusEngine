#pragma once

#include "Base/Types.h"

namespace Spline
{
    template <typename T>
    class InterpolatedStorage
    {
    public:
        explicit InterpolatedStorage(u32 step);
        void SetStep(u32 step);
        void SetInterpolation(const std::vector<T>& points, u32 step);

        void AddPortion(const std::vector<T>& portion, i32 position = -1);

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