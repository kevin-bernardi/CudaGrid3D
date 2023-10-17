#include <chrono>
#include <iostream>
#include <random>
#include <vector>

using namespace std;

void printMatrix(vector<vector<int>> matrix) {
    if (matrix.size() <= 0) {
        cout << "ERROR! Empty matrix" << endl;
        return;
    }

    int x = matrix.size();
    int y = matrix[0].size();

    for (int i = 0; i < x; i++) {
        for (int j = 0; j < y; j++) {
            if (j == y - 1)
                cout << matrix[i][j];
            else
                cout << matrix[i][j] << " | ";
        }

        cout << endl;
    }
}

void print3D(vector<vector<vector<int>>> matrix3D) {
    int n = matrix3D.size();
    if (n <= 0) {
        cout << "ERROR! Empty matrix" << endl;
        return;
    }

    cout << "3D Matrix Output" << endl;

    for (int i = 0; i < n; i++) {
        cout << "Layer " << i << endl;
        printMatrix(matrix3D[i]);
    }
}

void computeMidPoint(vector<vector<vector<int>>> matrix3D, bool debug = false) {
    float sumx, sumy, sumz, count;
    sumx = sumy = sumz = count = 0.0;
    for (int i = 0; i < matrix3D.size(); i++) {
        for (int j = 0; j < matrix3D[0].size(); j++) {
            for (int k = 0; k < matrix3D[0][0].size(); k++) {
                if (matrix3D[i][j][k] == 1) {
                    sumx += i;
                    sumy += j;
                    sumz += k;
                    count++;

                    if (debug)
                        cout << "Point ( " << i << ", " << j << ", " << k
                             << " )" << endl;
                }
            }
        }
    }

    cout << "Mid Point is at coordinates: ( " << sumx / count << ", " << sumy / count << ", " << sumz / count << " )" << endl;
}

void insertNRandomPoints(vector<vector<vector<int>>> &matrix3D, int n) {
    int x = matrix3D.size();
    int y = matrix3D[0].size();
    int z = matrix3D[0][0].size();

    for (int i = 0; i < n; i++) {
        int x_point = rand() % x;
        int y_point = rand() % y;
        int z_point = rand() % z;

        matrix3D[x_point][y_point][z_point] = 1;
    }
}

uint64_t timeSinceEpochMillisec() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch())
        .count();
}

int main() {
    uint64_t start = timeSinceEpochMillisec();

    srand(time(0));

    int sizeX = 1000;
    int sizeY = 1000;
    int sizeZ = 1000;

    vector<vector<vector<int>>> vector3d;

    for (int i = 0; i < sizeX; i++) {
        vector<vector<int>> a;
        for (int j = 0; j < sizeY; j++) {
            vector<int> b;
            for (int k = 0; k < sizeZ; k++) {
                b.push_back(0);
            }
            a.push_back(b);
        }
        vector3d.push_back(a);
    }

    cout << "Vector3D size: " << vector3d.size() << " x " << vector3d[0].size()
         << " x " << vector3d[0][0].size() << endl;

    insertNRandomPoints(vector3d, 1000);

    // print3D(vector3d);

    computeMidPoint(vector3d);

    uint64_t end = timeSinceEpochMillisec();

    cout << "Computed in " << (end - start) / 1000.0 << " seconds" << endl;
}