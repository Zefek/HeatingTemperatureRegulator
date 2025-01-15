#include <LiquidCrystal_I2C.h>
#include <Ds1302.h>
#include <Arduino.h>
#include "display.h"
  /* 19°*1**79°*15:34 */
  /* 21°******-17.66° */


    void Display::print2digits(uint8_t number) {
      if (number >= 0 && number < 10) {
        lcd->print("0");
      }
      lcd->print(number);
    }

    void Display::printTime()
    {
      Ds1302::DateTime now;
      rtc->getDateTime(&now);
      if(now.hour != this->hours || now.minute != this->minutes)
      {
        lcd->setCursor(11, 0);
        lcd->print("  ");
        lcd->setCursor(11, 0);
        print2digits(now.hour);
        //lcd->print(":");
        lcd->setCursor(14, 0);
        lcd->print("  ");
        lcd->setCursor(14, 0);
        print2digits(now.minute);
        this->minutes = now.minute;
        this->hours = now.hour;
        this->TimeChanged(this->hours, this->minutes);
      }
    }
    
    void Display::printOutTemperature()
    {
      //celkem 6 míst -10.00
      lcd->setCursor(9, 1);
      lcd->print("       ");
      lcd->setCursor(9, 1);
      if(outTemperature >= 0 && outTemperature < 10)
      {
        //8.98
        lcd->print("  ");
      }
      if((outTemperature > -10 && outTemperature < 0) || outTemperature >= 10)
      {
        //-8.89
        lcd->print(" ");
      }
      lcd->print(outTemperature);
      lcd->write((byte)1);
    }

    Display::Display(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows)
    {
      lcd = new LiquidCrystal_I2C(lcd_Addr, lcd_cols, lcd_rows);
    }

    void Display::Init(Ds1302* rtc, void (*timeChanged)(uint8_t, uint8_t))
    {
      lcd->init();
      lcd->createChar(0, onChar);
      lcd->createChar(1, celsiusChar);
      this->rtc = rtc;
      this->TimeChanged = timeChanged;
    }

    void Display::BackLight()
    {
      lcd->backlight();
    }

    void Display::SetRequiredTemperature(uint8_t requiredTemperature)
    {
      if(this->requiredTemperature != requiredTemperature)
      {
        this->requiredTemperature = requiredTemperature;
        lcd->setCursor(0, 0);
        lcd->print("   ");
        lcd->setCursor(0, 0);
        lcd->print(requiredTemperature);
        lcd->write((byte)1);
      }
    }
    
    void Display::SetOutTemperature(double outTemperature)
    {
      if(this->outTemperature != outTemperature)
      {
        this->outTemperature = outTemperature;
        printOutTemperature();
      }
    }

    void Display::SetInputTemperature(uint8_t inputTemperature)
    {
      if(this->inputTemperature != inputTemperature)
      {
        this->inputTemperature = inputTemperature;
        lcd->setCursor(7, 0);
        lcd->print("   ");
        lcd->setCursor(7, 0);
        lcd->print(inputTemperature);
        lcd->write((byte)1);
      }
    }

    void Display::SetCurrentHeatingTemperature(uint8_t currentHeatingTemperature)
    {
      if(this->currentHeatingTemperature != currentHeatingTemperature)
      {
        this->currentHeatingTemperature = currentHeatingTemperature;
        lcd->setCursor(0, 1);
        lcd->print("   ");
        lcd->setCursor(0, 1);
        lcd->print(currentHeatingTemperature);
        lcd->write((byte)1);
      }
    }
     
    void Display::SetMode(uint8_t mode)
    {
      this->mode = mode;
      lcd->setCursor(5, 0);
      if(mode == 0)
        lcd->print("O");
      if(mode == 1)
        lcd->print("A");
      if(mode == 2)
        lcd->print("T");
    }

    void Display::SetHeating(bool heatingOn)
    {
      if(this->heatingOn != heatingOn)
      {
        this->heatingOn = heatingOn;
        if(!heatingOn)
        {
          lcd->setCursor(4, 0);
          lcd->print(" ");
        }
      }
    }

    void Display::Blink()
    {
      if(blinkCount % 2 == 0)
      {
        lcd->setCursor(13, 0);
        lcd->print(":");
        if(heatingOn)
        {
          lcd->setCursor(4, 0);
          lcd->write((byte)0);
        }
      }
      else
      {
        lcd->setCursor(13, 0);
        lcd->print(" ");
        if(heatingOn)
        {
          lcd->setCursor(4, 0);
          lcd->print(" ");
        }
      }
      blinkCount++;
    }

    void Display::Print()
    {
      printTime();
      if(millis() - lastPrint > 1000)
      {
        Blink();
        lastPrint = millis();
      }
    }