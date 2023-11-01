#include "ListItem.h"

ListItem::ListItem(int hour, int minute, double temperature)
{
  this->hour = hour;
  this->minute = minute;
  this->temperature = temperature;
}

void ListItem::AddNext(ListItem *next)
{
  this->next = next;
}

int ListItem::GetHour()
{
  return this->hour;
}

int ListItem::GetMinute()
{
  return this->minute;
}

double ListItem::GetTemperature()
{
  return this->temperature;
}

ListItem* ListItem::GetNext()
{
  return next;
}