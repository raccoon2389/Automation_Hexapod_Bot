#include <Servo.h> //Servo 라이브러리를 추가

Servo c1;
Servo f1;
Servo t1;

Servo c2;
Servo f2;
Servo t2;

Servo c3;
Servo f3;
Servo t3;

Servo c4;
Servo f4;
Servo t4;

Servo c5;
Servo f5;
Servo t5;

Servo c6;
Servo f6;
Servo t6;

int value = 120; // 각도를 조절할 변수 value
int value2 = 120;
void setup()
{
    c1.attach(28);
    f1.attach(29);
    t1.attach(30);

    c2.attach(25);
    f2.attach(26);
    t2.attach(27);

    c3.attach(22);
    f3.attach(23);
    t3.attach(24);

    c4.attach(4);
    f4.attach(3);
    t4.attach(2);

    c5.attach(7);
    f5.attach(6);
    t5.attach(5);

    c6.attach(10);
    f6.attach(9);
    t6.attach(8);

    Serial.begin(9600); //시리얼 모니터 사용 고고
}

void loop()
{
    if (Serial.available()) //시리얼 모니터에 데이터가 입력되면
    {
        char in_data;            // 입력된 데이터를 담을 변수 in_data
        in_data = Serial.read(); //시리얼모니터로 입력된 데이터 in_data로 저장
        if (in_data == '1')      //입력된 데이터가 1이라면
        {
            value = 90; //각도를 30도 증가시킨다.
            value2=90;
        }
        else if(in_data =='2')
        {
            value = 120;
            value2 = 90;
        }
        else if(in_data =='3'){
            value = 90;
            value2 = 120;
        }
        else if(in_data =='4'){
            value = 120;
            value = 120;
        }
        
        Serial.print(value);
        c1.write(90);
        f1.write(value);
        t1.write(value);

        c2.write(90);
        f2.write(value2);
        t2.write(value2);

        c3.write(90);
        f3.write(value);
        t3.write(value);

        c4.write(90);
        f4.write(value2);
        t4.write(value2);

        c5.write(90);
        f5.write(value);
        t5.write(value);

        c6.write(90);
        f6.write(value2);
        t6.write(value2);

        // servo1.write(value); //value값의 각도로 회전. ex) value가 90이라면 90도 회전
        // delay(500);
        // servo2.write(value);
        // delay(500);
        // servo3.write(value);
        // delay(500);
        // servo4.write(value); //value값의 각도로 회전. ex) value가 90이라면 90도 회전
        // delay(500);
        // servo5.write(value);
        // delay(500);
        // servo6.write(value);
        // delay(500);
        // servo7.write(value); //value값의 각도로 회전. ex) value가 90이라면 90도 회전
        // delay(500);
        // servo8.write(value);
        // delay(500);
        // servo9.write(value);
        // delay(500);
        // Serial.print(value);
    }
}