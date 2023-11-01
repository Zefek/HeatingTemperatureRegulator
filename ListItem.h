class ListItem
{
   private:
     int hour;
     int minute;
     double temperature;
     ListItem* next;

  public:
    ListItem(int hour, int minute, double temperature);
    void AddNext(ListItem* next);
    int GetHour();
    int GetMinute();
    double GetTemperature();
    ListItem* GetNext();
};