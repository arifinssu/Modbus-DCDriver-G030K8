#include "MedianFilter.h"

MedianFilter* New_MedianFilter(int size, int seed)
{
   MedianFilter *self = (MedianFilter*) malloc(sizeof(MedianFilter));

   self->medFilterWin      = constrain(size, 3, 255);
   self->medDataPointer    = size >> 1;
   self->data              = (int*)     calloc (size, sizeof(int));
   self->sizeMap           = (uint8_t*) calloc (size, sizeof(uint8_t));
   self->locationMap       = (uint8_t*) calloc (size, sizeof(uint8_t));
   self->oldestDataPoint   = self->medDataPointer;
   self->totalSum          = size * seed;

   for(uint8_t i = 0; i < self->medFilterWin; i++)
   {
      self->sizeMap[i]     = i;
      self->locationMap[i] = i;
      self->data[i]        = seed;
   }

   return self;
}

int constrain(int v, int min, int max) 
{
    if(v < min) return min;
    else if(max < v) return max;
    else return v;
}

int MedianFilter_In(MedianFilter *self, const int value)
{
   bool dataMoved = false;
   const uint8_t rightEdge = self->medFilterWin - 1;

   self->totalSum += value - self->data[self->oldestDataPoint];

   self->data[self->oldestDataPoint] = value;

   // SORT LEFT (-) <======(n) (+)
   if(self->locationMap[self->oldestDataPoint] > 0)
   {
      for(uint8_t i = self->locationMap[self->oldestDataPoint]; i > 0; i--)
      {
         uint8_t n = i - 1;

         if(self->data[self->oldestDataPoint] < self->data[self->sizeMap[n]])
         {
            self->sizeMap[i] = self->sizeMap[n];
            self->locationMap[self->sizeMap[n]]++;

            self->sizeMap[n] = self->oldestDataPoint;
            self->locationMap[self->oldestDataPoint]--;

            dataMoved = true;
         }
         else
         {
            break;
         }
      }
   }

   // SORT RIGHT (-) (n)======> (+)
   if(!dataMoved && self->locationMap[self->oldestDataPoint] < rightEdge)
   {
      for(int i = self->locationMap[self->oldestDataPoint]; i < rightEdge; i++)
      {
         int n = i + 1;

         if(self->data[self->oldestDataPoint] > self->data[self->sizeMap[n]])
         {
            self->sizeMap[i] = self->sizeMap[n];
            self->locationMap[self->sizeMap[n]]--;

            self->sizeMap[n] = self->oldestDataPoint;
            self->locationMap[self->oldestDataPoint]++;
         }
         else
         {
            break;
         }
      }
   }
   self->oldestDataPoint++;
   if(self->oldestDataPoint == self->medFilterWin) self->oldestDataPoint = 0;

   return self->data[self->sizeMap[self->medDataPointer]];
}

int MedianFilter_Out(MedianFilter *self)
{
   return self->data[self->sizeMap[self->medDataPointer]];
}

int MedianFilter_GetMin(MedianFilter *self)
{
   return self->data[self->sizeMap[ 0 ]];
}

int MedianFilter_GetMax(MedianFilter *self)
{
   return self->data[self->sizeMap[ self->medFilterWin - 1 ]];
}

int MedianFilter_GetMean(MedianFilter *self)
{
   return self->totalSum / self->medFilterWin;
}

void MedianFilter_Clear(MedianFilter *self)
{
   free(self->data);
   free(self->sizeMap);
   free(self->locationMap);
   free(self);
}