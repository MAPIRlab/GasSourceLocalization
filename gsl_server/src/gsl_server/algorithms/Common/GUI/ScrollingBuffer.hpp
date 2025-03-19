#pragma once
#if USE_GUI

#include <ament_imgui/ament_imgui.h>

namespace GSL::GUI
{

    // utility structure for realtime plot
    struct ScrollingBuffer
    {
        int MaxSize;
        int IndexOfLast;
        ImVector<ImVec2> Data;
        ScrollingBuffer(int max_size = 2000)
        {
            MaxSize = max_size;
            IndexOfLast = 0;
            Data.reserve(MaxSize);
        }
        void AddPoint(float x, float y)
        {
            if (Data.size() < MaxSize)
                Data.push_back(ImVec2(x, y));
            else
            {
                Data[IndexOfLast] = ImVec2(x, y);
                IndexOfLast = (IndexOfLast + 1) % MaxSize;
            }
        }
        void Erase()
        {
            if (Data.size() > 0)
            {
                Data.shrink(0);
                IndexOfLast = 0;
            }
        }
    };
} // namespace GSL::GUI
#endif