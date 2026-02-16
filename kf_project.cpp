#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <thread> // 딜레이를 주기 위해 추가 (실감나게 하려고)
#include <chrono> // 시간 관련
#include"KalmanFilter.h"
using namespace std;
using namespace Eigen;


int main() {
    try {
        // [1] 센서 연결 (여기서는 파일 열기로 대체)
        string meas_path = "C:\\Users\\k0min\\Downloads\\measurement.csv";
        ifstream file(meas_path);

        if (!file.is_open()) {
            cout << "Error: Sensor(File) connect failed!" << endl;
            return -1;
        }

        // [2] 첫 데이터 읽어서 초기화
        string line;
        getline(file, line); // 헤더(k,mx,my) 건너뛰기

        // 첫 번째 데이터 파싱
        getline(file, line);
        stringstream ss(line);
        string token;
        getline(ss, token, ','); // k 버림
        getline(ss, token, ','); double mx = stod(token);
        getline(ss, token, ','); double my = stod(token);

        // KF 객체 생성 및 초기화
        KalmanFilter kf(1.0, 2.0);
        kf.init(mx, my);

        cout << "=== Real-time Tracking Started ===" << endl;

        // [3] 무한 루프 (데이터가 끝날 때까지)
        // 실제 로봇이라면 while(true) 해놓고 센서 대기
        int k = 1;
        while (getline(file, line)) {

            // --- 1. 센서 데이터 수신 (Parsing) ---
            stringstream ss2(line);
            getline(ss2, token, ','); // k (파일에 있는 k는 무시하고 우리가 셈)
            getline(ss2, token, ','); double mx = stod(token);
            getline(ss2, token, ','); double my = stod(token);
            Vector2d z_meas;
            z_meas << mx, my;

            // --- 2. 칼만 필터 수행 (핵심!) ---
            kf.Predict();
            kf.Update(z_meas);
            Vector4d result = kf.State();

            // --- 3. 결과 출력 (실시간 로그) ---
            // 파일 저장이 아니라 화면에 바로바로 띄웁니다.
            printf("[Step %d] Meas:(%.2f, %.2f) -> Est:(%.2f, %.2f)\n",
                k, mx, my, result(0), result(1));

            // --- 4. 리얼타임 흉내내기 (선택사항) ---
            // 실제 데이터 간격(dt=1.0초)처럼 보이게 0.1초씩 쉬어봅니다.
           
            this_thread::sleep_for(chrono::milliseconds(50));

            k++;
        }

        cout << "=== Tracking Finished ===" << endl;

    }
    catch (const exception& e) {
        cerr << e.what() << endl;
        return 1;
    }
    return 0;
}

