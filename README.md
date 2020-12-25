# donation robot on raspberryPi

### 프로젝트 소개 : HanyangUni_IcPbl CrashLab - 기부 로봇 제작하기
                
                
                
### 프로젝트 명 : 럭키(포켓몬스터)




### 4조 기부로봇 소개 
    Covid-19로 인해 고생하는 의료진들을 위해 진심어린응원을 기부를 유도

  
  
  

## 시나리오
![image](https://user-images.githubusercontent.com/70446214/103131597-ba7a5800-46e4-11eb-9454-8bc153989066.png)

## 주요 기능 구현 방식

#### 1. 모터 제어 
    양쪽 모터는 pid 제어를 통해 원하는 속도을 낼 수 있도록 제어

#### 2. Tarcking

![image](https://user-images.githubusercontent.com/70446214/103130856-c3b5f580-46e1-11eb-8ced-a4112b7903e4.png)

- 사람을 Tracking 할 시, 로봇과 사람간의 거리(d) 와 각도(th) 자체를 error로 취급하여
  pd제어
- d 와 th에 (error) 비례하는 선속도와 각속도를 동시에 가해줌

#### 3. 복귀

- 로봇은 처음 시작한 자신의 위치를 'world'Frame으로 생성하며, 럭키 자신은
로봇의 중앙을 중심으로 'base'Frame으로 생성

- 엔코더를 이용해 로봇이 실제로 이동한 선속도와 각속도를 기반으로 baseFrame을 업데이트

![ezgif com-gif-maker (5)](https://user-images.githubusercontent.com/70446214/103131682-10e79680-46e5-11eb-95bb-ccb9fd45daa6.gif)

(1) 로봇이 world Frame을 기준으로 4m 이상 벗어난 경우

(2) 사람이 응답하지 않는 경우

(3) 인터랙션을 완료한 경우

위 의 경우 로봇은 원점으로 복귀

## 역할

![image](https://user-images.githubusercontent.com/70446214/103132389-b996f580-46e7-11eb-92e8-fd66c3b71222.png)



