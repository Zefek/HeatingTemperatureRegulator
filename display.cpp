#include <LiquidCrystal_I2C.h>
#include <Ds1302.h>
#include <Arduino.h>
#include "display.h"
  /* 19°*1A*79°*15:34 */
  /* 21°*123°*-17.66° */


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
      }
    }
    
    void Display::printOutTemperature(bool blink)
    {
      if(blink)
      {
        lcd->setCursor(9, 1);
        lcd->print("       ");
      }
      else
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
    }

    Display::Display(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows)
    {
      lcd = new LiquidCrystal_I2C(lcd_Addr, lcd_cols, lcd_rows);
    }

    void Display::Init(Ds1302* rtc)
    {
      intializing = true;
      lcd->init();
      lcd->backlight();
      lcd->createChar(0, onChar);
      lcd->createChar(1, celsiusChar);
      this->rtc = rtc;
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Inicializace...");
    }

    void Display::SetRequiredTemperature(uint8_t requiredTemperature)
    {
      if(this->requiredTemperature != requiredTemperature || forcePrint)
      {
        this->requiredTemperature = requiredTemperature;
        if(!intializing)
        {
          lcd->setCursor(0, 0);
          lcd->print("   ");
          lcd->setCursor(0, 0);
          lcd->print(requiredTemperature);
          lcd->write((byte)1);
        }
      }
    }
    
    void Display::SetOutTemperature(double outTemperature)
    {
      if(this->outTemperature != outTemperature || forcePrint)
      {
        this->outTemperature = outTemperature;
        if(!intializing)
        {
          printOutTemperature(false);
        }
      }
      outsideTemperatureWasSet = true;
    }

    void Display::SetWasteGasTemperature(int wasteGasTemperature)
    {
      if(this->wasteGasTemperature != wasteGasTemperature || forcePrint)
      {
        this->wasteGasTemperature = wasteGasTemperature;
        if(!intializing)
        {
          lcd->setCursor(4, 1);
          lcd->print("    ");
          lcd->setCursor(4, 1);
          if(wasteGasTemperature < 10)
          {
            lcd->print("  ");
          }
          else if(wasteGasTemperature < 100)
          {
            lcd->print(" ");
          }
          lcd->print(wasteGasTemperature);
          lcd->write((byte)1);
        }
      }
    }

    void Display::SetInputTemperature(uint8_t inputTemperature)
    {
      if(this->inputTemperature != inputTemperature || forcePrint)
      {
        this->inputTemperature = inputTemperature;
        if(!intializing)
        {
          lcd->setCursor(7, 0);
          lcd->print("   ");
          lcd->setCursor(7, 0);
          lcd->print(inputTemperature);
         lcd->write((byte)1);
        }
      }
    }

    void Display::SetCurrentHeatingTemperature(uint8_t currentHeatingTemperature)
    {
      if(this->currentHeatingTemperature != currentHeatingTemperature || forcePrint)
      {
        this->currentHeatingTemperature = currentHeatingTemperature;
        if(!intializing)
        {
          lcd->setCursor(0, 1);
          lcd->print("   ");
          lcd->setCursor(0, 1);
          lcd->print(currentHeatingTemperature);
          lcd->write((byte)1);
        }
      }
    }
     
    void Display::SetMode(uint8_t mode)
    {
      this->mode = mode;
      if(!intializing)
      {
        PrintMode(mode, false);
      }
    }

    void Display::PrintMode(uint8_t mode, bool blink)
    {
      lcd->setCursor(5, 0);
      if(mode == 0)
        lcd->print("O");
      if(mode == 1)
        lcd->print("A");
      if(mode == 2)
      {
        if(blink)
        {
          lcd->print(" ");
        }
        else
        {
          lcd->print("T");
        }
      }
    }

    void Display::SetHeating(bool heatingOn)
    {
      if(this->heatingOn != heatingOn || forcePrint)
      {
        this->heatingOn = heatingOn;
        if(!intializing)
        {
          if(!heatingOn)
          {
            lcd->setCursor(4, 0);
            lcd->print(" ");
          }
        }
      }
    }

    void Display::Blink()
    {
      if(!shouldBlink)
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
      if(!outsideTemperatureWasSet)
      {
        printOutTemperature(shouldBlink);
      }
      if(mode == 2 && !thermostat)
      {
        PrintMode(mode, shouldBlink);
      }
      shouldBlink = !shouldBlink;
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

    void Display::SetThermostat(bool thermostat)
    {
      this->thermostat = thermostat;
      if(!intializing)
      {
        PrintMode(mode, false);
      }
    }

    void Display::EndInitialize()
    {
      intializing = false;
      forcePrint = true;
      lcd->clear();
      SetRequiredTemperature(this->requiredTemperature);
      SetOutTemperature(this->outTemperature);
      SetWasteGasTemperature(this->wasteGasTemperature);
      SetInputTemperature(this->inputTemperature);
      SetCurrentHeatingTemperature(this->currentHeatingTemperature);
      SetMode(this->mode);
      SetHeating(this->heatingOn);
      SetThermostat(this->thermostat);
      this->Print();
      forcePrint = false;
    }