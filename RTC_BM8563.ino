#include "M5CoreInk.h"
#include <Wire.h>
#include "icon.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "SHT3X.h"
#include "esp_adc_cal.h"
#include "time.h"

SHT3X sht30;
Adafruit_BMP280 bme;

Ink_Sprite TimePageSprite(&M5.M5Ink);

RTC_TimeTypeDef RTCtime;
RTC_DateTypeDef RTCDate;

uint8_t minutes = -1;

float tmp = 0.0;
float hum = 0.0;
float pressure = 0.0;
float tmp2 = 0.0;

void setupTime(){
  
  RTCtime.Hours = 19;
  RTCtime.Minutes = 23;
  RTCtime.Seconds = 0;
  M5.rtc.SetTime(&RTCtime);
  
  RTCDate.Year = 2021;
  RTCDate.Month = 1;
  RTCDate.Date = 22;
  M5.rtc.SetDate(&RTCDate);
}


float getBatVoltage()
{
    analogSetPinAttenuation(35,ADC_11db);
    esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 3600, adc_chars);
    uint16_t ADCValue = analogRead(35);
    
    uint32_t BatVolmV  = esp_adc_cal_raw_to_voltage(ADCValue,adc_chars);
    free(adc_chars);
    float BatVol = float(BatVolmV) * 25.1 / 5.1 / 1000;
    return BatVol;
}

int last_height = 0;
int last_left = 0;
void drawImageToSprite(int posX,int posY,image_t* imagePtr,Ink_Sprite* sprite)
{
    sprite->drawBuff(   posX, posY,
                        imagePtr->width, imagePtr->height, imagePtr->ptr);
    last_height = posY + imagePtr->height;
    last_left = posX + imagePtr->width;
}

void drawTime( RTC_TimeTypeDef *time )
{
    int height_start = 0;
    drawImageToSprite(10 ,height_start,&num55[time->Hours/10],&TimePageSprite);
    drawImageToSprite(50 ,height_start,&num55[time->Hours%10],&TimePageSprite);
    drawImageToSprite(90 ,height_start,&num55[10],&TimePageSprite);
    drawImageToSprite(110,height_start,&num55[time->Minutes/10],&TimePageSprite);
    drawImageToSprite(150,height_start,&num55[time->Minutes%10],&TimePageSprite);
}

void drawDate( RTC_DateTypeDef *date)
{
    int height_start = last_height + 5;
    int posX = 15, num = 0;
    for( int i = 0; i < 4; i++ )
    {
        num = ( date->Year / int(pow(10,3-i)) % 10);
        drawImageToSprite(posX,height_start,&num18x29[num],&TimePageSprite);
        posX += 17;
    }
    drawImageToSprite(posX,height_start,&num18x29[10],&TimePageSprite);posX += 17;

    drawImageToSprite(posX,height_start,&num18x29[date->Month / 10 % 10],&TimePageSprite);posX += 17;
    drawImageToSprite(posX,height_start,&num18x29[date->Month % 10],&TimePageSprite);posX += 17;

    drawImageToSprite(posX,height_start,&num18x29[10],&TimePageSprite);posX += 17;

    drawImageToSprite(posX,height_start,&num18x29[date->Date  / 10 % 10 ],&TimePageSprite);posX += 17;
    drawImageToSprite(posX,height_start,&num18x29[date->Date  % 10 ],&TimePageSprite);posX += 17;
}

void drawTemperatureHumidity(float temperature, float humidity) {
    int height_start = last_height + 10;
    char buff[50];
    int len=sprintf(buff, "%d.", (int)temperature);    
    TimePageSprite.drawString(10, height_start, buff, &AsciiFont24x48);   
    //TimePageSprite.drawString(10+len-16, height_start + 48 - 16, buff, &AsciiFont8x16);   
    drawImageToSprite(10+(len*24)-10, height_start + 48 - 34,&num18x29[(int)(temperature*100) % 10],&TimePageSprite);
    TimePageSprite.drawString(10+(len*24)-10, height_start - 8, "o", &AsciiFont8x16);     
    TimePageSprite.drawString(10+(len*24)-2, height_start, "C", &AsciiFont8x16);     

    len=sprintf(buff, "%d", (int)humidity);    
    TimePageSprite.drawString(TimePageSprite.width() - len * 24 - (8 * 2), height_start, buff, &AsciiFont24x48);     
    TimePageSprite.drawString(TimePageSprite.width() - 8, height_start + 4, "%", &AsciiFont8x16);     
    TimePageSprite.drawString(TimePageSprite.width() - (8 * 2), height_start + 48 - 20, "RH", &AsciiFont8x16);     
    //sprintf(buff, "%.1f°C", humidity);         
    last_height=height_start+43;
    
}

void drawPressure(float atmo_pressure) {
  int height_start = last_height;
  char buff[50];
  int len=sprintf(buff, "%d.%d", (int)atmo_pressure, (int)(atmo_pressure * 100) % 100);
  int xPressure = TimePageSprite.width() / 2 - (len*24) / 2 - 2*8;
  TimePageSprite.drawString(xPressure, height_start, buff, &AsciiFont24x48);
  TimePageSprite.drawString(xPressure + len * 24, height_start + 48 - 20, "hPa", &AsciiFont8x16);     
  float voltage = getBatVoltage();
  len=sprintf(buff, "%d.%d V", (int)voltage, (int)(voltage * 100) % 100);
  TimePageSprite.drawString(5, TimePageSprite.height() - 16, buff, &AsciiFont8x16);
}

            
void flushTimePage()
{
    while(1)
    {
        pressure = bme.readPressure() / 100.0;
        tmp2 = bme.readTemperature();
        if(sht30.get()==0){
          tmp = sht30.cTemp;
          hum = sht30.humidity;
        }
        Serial.printf("Temperatur: %2.1f*C Temperature2: %2.1f*C  Hum: %0.2f%%  Pressure: %0.2fhPa\r\n", tmp, tmp2, hum, pressure );
        M5.rtc.GetTime(&RTCtime);
        if( minutes != RTCtime.Minutes )
        {
            M5.rtc.GetTime(&RTCtime);
            M5.rtc.GetDate(&RTCDate);
            
            if( RTCtime.Minutes % 10 == 0 && millis() > 60000)
            {
                M5.M5Ink.clear();
                TimePageSprite.clear( CLEAR_DRAWBUFF | CLEAR_LASTBUFF );                
            }
            drawTime(&RTCtime);
            drawDate(&RTCDate);
            drawTemperatureHumidity(tmp, hum);
            drawPressure(pressure);
            TimePageSprite.pushSprite();
            // Restart esp32 on the next minute
            M5.rtc.GetTime(&RTCtime);
            minutes = RTCtime.Minutes;
            M5.shutdown(60-RTCtime.Seconds);
        }
        delay(1000);
        M5.update();
    }
}

void setup() {
    M5.begin();
    
    Wire.begin();
    
    if( M5.BtnMID.isPressed())
    {
      setupTime();
    }
    if( !M5.M5Ink.isInit())
    {
        Serial.printf("Ink Init failed");
        while (1) delay(100);   
    }
    while (!bme.begin(0x76)){  // GPIO_NUM_32
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    }
    
    M5.M5Ink.clear();

    //creat ink refresh Sprite
    if( TimePageSprite.creatSprite(0,0,200,200, true) != 0 )
    {
        Serial.printf("Ink Sprite create failed");
    }
}

void loop() {
  flushTimePage();
  M5.update();
}
