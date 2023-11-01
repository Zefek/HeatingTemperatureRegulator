Heating equithermal regulator for Arduino

Regulator works with ESBE ARA 651 valve.

Input temperature is water temperature which input into valve.

Celsius is water temperature which is mixed by valve. Value si computed as average for each second.

Value is water temperature which is computed by temperature slope. Slope needs to be computed by building temperature losses.

Outside temperature is outside temperature. This temperature is give by wireless SOLIGHT TE8S sensor. Outside temperature is computed by sliding average for one hour. 

Regulator uses pump. Pump is on when input water temperature is sufficient or outside temperature is less then 13°C. If pump is off ESBE valve is closed to prevent gravity mode in heating system.

