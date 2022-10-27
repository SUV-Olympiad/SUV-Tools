# Mission

- A, B, C 각각 최대 10대의 드론에게 미션 업로드 가능

## Run

- python3 mission_control_A.py {Number of vehicle}

## conf

- LLH 좌표계 사용

#### 1. dst_{VertiPort}.csv

- 각 VertiPort에 여러 landing point가 존재
    #### format
    ```csv
    latitude,longitude,height
    ```
  
#### 2.point_{VertiPort}.csv 

- VertiPort의 꼭짓점 좌표 
  #### format
    ```csv
    latitude,longitude,height
    ```

## log
- 미션이 종료한 기체의 비행로그가 기업별로 저장