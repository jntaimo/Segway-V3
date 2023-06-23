struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
    bool success;
};

void imuSetup(void);
EulerAngles readIMU();

void setReports(void);
void printEuler(EulerAngles angles);
EulerAngles ToEulerAngles(Quaternion q);
