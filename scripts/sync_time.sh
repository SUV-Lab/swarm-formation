#!/bin/bash

# NTP 시간 동기화 스크립트
# LTE 인터넷 연결이 필요합니다

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}NTP 시간 동기화를 시작합니다...${NC}"

# ntpdate 설치 확인
if ! command -v ntpdate &> /dev/null; then
    echo -e "${YELLOW}ntpdate가 설치되어 있지 않습니다. 설치를 시도합니다...${NC}"
    sudo apt-get update && sudo apt-get install -y ntpdate
    if [ $? -ne 0 ]; then
        echo -e "${RED}ntpdate 설치 실패${NC}"
        exit 1
    fi
fi

# 인터넷 연결 확인
echo -e "${YELLOW}인터넷 연결을 확인합니다...${NC}"
if ! ping -c 1 8.8.8.8 &> /dev/null; then
    echo -e "${RED}인터넷 연결이 없습니다. LTE 연결을 확인하세요.${NC}"
    exit 1
fi
echo -e "${GREEN}인터넷 연결 확인됨${NC}"

# 현재 시간 출력
echo -e "${YELLOW}동기화 전 시간: $(date)${NC}"

# NTP 서버 목록
NTP_SERVERS=(
    "time.google.com"
    "time.bora.net"
    "time.kriss.re.kr"
    "pool.ntp.org"
)

# NTP 서버를 순서대로 시도
SUCCESS=false
for server in "${NTP_SERVERS[@]}"; do
    echo -e "${YELLOW}NTP 서버 ${server}에서 시간 동기화 시도...${NC}"
    if sudo ntpdate -u "$server"; then
        SUCCESS=true
        echo -e "${GREEN}${server}에서 시간 동기화 성공!${NC}"
        break
    else
        echo -e "${RED}${server} 동기화 실패, 다음 서버 시도...${NC}"
    fi
done

if [ "$SUCCESS" = false ]; then
    echo -e "${RED}모든 NTP 서버에서 동기화 실패${NC}"
    exit 1
fi

# 동기화 후 시간 출력
echo -e "${GREEN}동기화 후 시간: $(date)${NC}"

# 하드웨어 시계에도 반영 (선택사항)
echo -e "${YELLOW}하드웨어 시계에 시간을 기록합니다...${NC}"
sudo hwclock --systohc
if [ $? -eq 0 ]; then
    echo -e "${GREEN}하드웨어 시계 업데이트 완료${NC}"
else
    echo -e "${YELLOW}하드웨어 시계 업데이트 실패 (일부 시스템에서는 정상)${NC}"
fi

echo -e "${GREEN}시간 동기화 완료!${NC}"
