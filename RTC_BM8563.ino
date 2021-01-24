#include "M5CoreInk.h"
#include <Wire.h>
#include "icon.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "SHT3X.h"
#include "esp_adc_cal.h"

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

//
//void drawTimePage( )
//{
//    M5.rtc.GetTime(&RTCtime);
//    drawTime(&RTCtime);
//    minutes = RTCtime.Minutes;
//    M5.rtc.GetDate(&RTCDate);
//    drawDate(&RTCDate);
//    TimePageSprite.pushSprite();
//}

void drawTemperatureHumidity(float temperature, float humidity) {
    int height_start = last_height + 10;
    drawImageToSprite(10 ,height_start,&num55[((int)temperature/10)],&TimePageSprite);
    drawImageToSprite(50 ,height_start,&num55[((int)temperature)%10],&TimePageSprite);
    
    drawImageToSprite(last_left+2 ,height_start,&num55[((int)humidity/10)],&TimePageSprite);
    drawImageToSprite(last_left+2 ,height_start,&num55[((int)humidity)%10],&TimePageSprite);
  
}

void drawPressure(float atmo_pressure) {

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
            
            if( RTCtime.Minutes % 10 == 0 )
            {
                M5.M5Ink.clear();
                TimePageSprite.clear( CLEAR_DRAWBUFF | CLEAR_LASTBUFF );                
            }
            drawTime(&RTCtime);
            drawDate(&RTCDate);
            drawTemperatureHumidity(tmp, hum);
            drawPressure(pressure);
            TimePageSprite.pushSprite();
            minutes = RTCtime.Minutes;
            // Restart esp32 on the next minute
        }
        delay(1000);
        M5.update();
    }
    M5.M5Ink.clear();
    TimePageSprite.clear( CLEAR_DRAWBUFF | CLEAR_LASTBUFF );
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
    delay(1000);
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
