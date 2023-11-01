#include "ListItem.h"

class TemperatureList
{
   private:
     int count;
     double averageTemperature;
     ListItem* first;
     ListItem* last;
     bool HasMoreThanHour();

  public:
    void Add(int hour, int minute, double temperature);
    double GetAverageTemperature();
};