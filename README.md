# HerkuleX_ROS2
## HerkuleX ROS2(jazzy) Package
### Model number: 0:DRS-0101, 1:DRS-0102, 2:DRS-0201, 3:DRS-0401, 4:DRS-0402, 5:DRS-0601, 6:DRS-0602 


[Topic] (ID를 1번으로 설정한 HerkuleX 0602를 예시으로 설명합니다.)
- Info_RAM_ID1, Info_EEP_ID1

[service] 
- Register_cmd : 아래 10개의 command를 선택하여 사용가능하며, 모델번호와 ID값을 input으로 사용합니다.
>> RAM_RegisterData_Read_All: RAM의 모든 데이터 읽기  
>> EEP_RegisterData_Read_All: EPP의 모든 데이터 읽기  
>> RAM_RegisterData_Read: RAM의 특정 주소번지 읽기(Input에 주소 값 추가)  
>> EEP_RegisterData_Read: EEP의 특정 주소번지 읽기(Input에 주소 값 추가)  
>> RAM_RegisterData_Write: RAM 주소번지에 데이터 쓰기 (Input에 주소 값과 수정하는 데이터 값 추가)   
>> EEP_RegisterData_Write: EEP 주소번지에 데이터 쓰기 (Input에 주소 값과 수정하는 데이터 값 추가)  
>> SERVO_ON: 토크(Torque)ON   
>> SERVO_OFF: 토크(Torque)OFF   
>> BRAKE_ON: 토크(Torque)ON 자세 유지  
>> ERROR_CLEAR: 에러 클리어  

- PositionMove_cmd
>> 위치제어모드 (ADC카운트 값 기준으로 위치이동)   
- VelocityMove_cmd
>> 속도 제어모드 (ADC카운트 값 기준으로 속도제어)   
- IjogMove_cmd
>> 최대 43개의 HerkuleX에 JOG모드 전송이 가능한 모드이며, 각각의 HerkuleX에 위치와 시간(playtime)할당 가능
- SjogMove_cmd
>> 최대 53개의 HerkuleX에 JOG모드 전송이 가능한 모드이며, 각각의 HerkuleX에는 모두 동일한 시간을 가짐
- AngleMove_cmd
>> 사용자가 원하는 각도위치로 이동하는 기능  
>> unit의 값이 true이면 Degree단위, false면 Radian 단위로 제어  




### AngleMove_cmd 예시 (0.0 deg 이동)  
![Image](https://github.com/user-attachments/assets/b881dc5a-61f4-4cc8-9e27-f5fcf019069e)

### AngleMove_cmd 예시 (90.0 deg 이동)  
![Image](https://github.com/user-attachments/assets/8e8149b3-cb1e-430e-9ec1-8826ca599ae3)

### AngleMove_cmd 예시 (-1.5708 rad 이동)  
![Image](https://github.com/user-attachments/assets/79c58a6e-eee7-4d63-a29e-67b4574163b7)

### EEP 데이터 읽기 서비스 예시  
![Image](https://github.com/user-attachments/assets/7622a8e9-a7c0-4961-806e-adcd63adbdcc)
### EEP Topic ehco  
![Image](https://github.com/user-attachments/assets/139c6a13-878d-48ea-b746-14ddb3ea4d4b)
### RAM 데이터 읽기 서비스 예시  
![Image](https://github.com/user-attachments/assets/d521adab-bcde-4fe8-be90-cf432d56b1d2)
### RAM Topic echo  
![Image](https://github.com/user-attachments/assets/cf578090-2954-492e-a278-928f23890b97)
