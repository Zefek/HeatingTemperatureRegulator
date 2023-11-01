#include "TemperatureList.h"
#include <Arduino.h>

void TemperatureList::Add(int hour, int minute, double temperature)
{
  if(!this->first)
  {
    this->first = new ListItem(hour, minute, temperature);
    this->averageTemperature = temperature;
    this->last = this->first;
    this->count = 1;
  }
  else
  {
    if(HasMoreThanHour())
    {
      this->averageTemperature = ((this->averageTemperature * count) - this->first->GetTemperature()) / (this->count - 1);
      this->count--;  
      this->first = this->first->GetNext();
    }
    this->last->AddNext(new ListItem(hour, minute, temperature));
    this->averageTemperature = ((this->averageTemperature * count) + temperature) / (this->count + 1);
    this->count++;
    this->last = this->last->GetNext();
  }
}

bool TemperatureList::HasMoreThanHour()
{
  
  int lastminutes = this->last->GetHour() * 60 + this->last->GetMinute();
  int firstminutes = this->first->GetHour() * 60 + this->first->GetMinute();
  int totalMinutes = lastminutes - firstminutes;
  while(totalMinutes < 0)
  {
    totalMinutes += 1440;
  }
  return totalMinutes / (double)60 > 1;
}

double TemperatureList::GetAverageTemperature()
{
  return this->averageTemperature;
}