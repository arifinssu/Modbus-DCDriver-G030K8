#ifndef MedianFilter_h
#define MedianFilter_h

#ifdef __cplusplus
extern "C"
{
#endif

    #include <stdbool.h>
    #include <stdio.h>

    typedef struct MedianFilter MedianFilter;

    struct MedianFilter
    {
        uint8_t medFilterWin;
        uint8_t medDataPointer;
        int     * data;
        uint8_t * sizeMap;
        uint8_t * locationMap;
        uint8_t oldestDataPoint;
        int32_t totalSum;
    };

    MedianFilter* New_MedianFilter(int size, int seed);
    int MedianFilter_In(MedianFilter *self, const int value);
    int MedianFilter_Out(MedianFilter *self);
    int MedianFilter_GetMin(MedianFilter *self);
    int MedianFilter_GetMax(MedianFilter *self);
    int MedianFilter_GetMean(MedianFilter *self);
    void MedianFilter_Clear(MedianFilter *self);

#ifdef __cplusplus
}
#endif

#endif
