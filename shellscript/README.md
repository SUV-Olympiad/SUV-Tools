# 실행 메뉴얼

## 실행 전 필요한 것
- [mywww5371/suv:1.6](https://hub.docker.com/r/mywww5371/suv/tags)
- [PX4-Autopilot](https://github.com/SUV-Olympiad/PX4-Autopilot)
- [SUV-GCS](https://github.com/SUV-Olympiad/SUV-GCS)
- [SUV-Tools](https://github.com/SUV-Olympiad/SUV-Tools)
- [models](https://drive.google.com/drive/folders/1iQrGri4qP_nPKJhN0nCnCg6VGx8K2sPp?usp=sharing)

- 3가지 repository + models 한 `/home/$USER/olympiad`에 clone
- working dir을 변경하고 싶은 경우 [docker](./docker)에 있는 모든 run_*.sh에 `-v /home/$USER/olympiad:/home/suv/olympiad` 옵션 자신이 원하는 경로로 수정


### run_all.sh
- SUV 시스템을 쉽게 실행 시킬 수 있도록 작성한 스크립트
- PX4, Gazebo, SUV-GCS 실행 및 QGC 3개, 자동 미션 업로드 스크립트 실행

### run_leap.sh
- LeapMotion을 이용하기 위해서는 해당 스크립트 실행
- 실행 전 LeapMotion 연결 필수

