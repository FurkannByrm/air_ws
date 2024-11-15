float32 battery_voltage  # packVoltage      [Bataryanın toplam voltajı]
float32 battery_current  # packCurrent      [Bataryadan çekilen ya da bataryaya gelen akım (0.1 Amper). Pozitifse şarj oluyor, negatifse deşarj oluyor.]
float32 battery_soc      # packSOC          [State of Charge, bataryanın şarj durumunu yüzdesel olarak ifade eder.]

float32 temp_average     # tempAverage      [Batarya sıcaklıklarının ortalaması.]

string charge_status     ## chargeFetState   [Şarj FET (Field Effect Transistor) durumu (true: açık, false: kapalı).]
                         # disChargeFetState[Deşarj FET durumu (true: açık, false: kapalı).]