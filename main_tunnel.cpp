#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>
#include"KalmanFilter.h"
using namespace std;

int main() {
    // 1. 센서 데이터 파일 열기
    
    string meas_path = "C:\\Users\\k0min\\Downloads\\measurement.csv";
    ifstream file(meas_path);

    if (!file.is_open()) {
        cerr << "Error: Cannot open measurement.csv" << endl;
        return -1;
    }

    // 2. 초기화
    string line;
    getline(file, line); // 헤더 스킵

    // 첫 번째 데이터 읽기
    getline(file, line);
    stringstream ss(line);
    string token;
    getline(ss, token, ',');
    getline(ss, token, ','); double mx = stod(token);
    getline(ss, token, ','); double my = stod(token);

    // KF 객체 생성
    KalmanFilter kf(1.0, 2.0);
    kf.init(mx, my);

    // 저장할 파일
    ofstream out("estimate_tunnel.csv");
    out.precision(15);
    out << "k,mx,my,est_px,est_py" << endl;

    cout << "=== Tunnel Simulation Started ===" << endl;

    // 3. 루프 시작
    int k = 1;
    while (getline(file, line)) {
        stringstream ss2(line);
        getline(ss2, token, ',');
        getline(ss2, token, ','); double zx = stod(token);
        getline(ss2, token, ','); double zy = stod(token);

        Vector2d z_meas;
        z_meas << zx, zy;

        // 터널 시뮬레이션 로직 

        // 1. 예측 (Predict)은 항상 수행! (관성 비행)
        kf.Predict();

        // 2. 업데이트 (Update)는 조건부 수행
        if (k >= 100 && k <= 150) {
            // 터널 진입! 
            // 센서 데이터(z_meas)를 아예 쓰지 않음 (Update 생략)
            // "나는 내 예측을 믿고 간다"
        }
        else {
            
            kf.Update(z_meas);
        }
   

        // 결과 저장
        Vector4d res = kf.State();
        out << k << "," << zx << "," << zy << "," << res(0) << "," << res(1) << "\n";

        k++;
    }

    out.close();
    cout << "Done! Saved 'estimate_tunnel.csv'" << endl;

    return 0;
}